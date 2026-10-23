// Unit tests of localizers/last_pose_store.*: the file the next AUTO session
// takes its boot guess from. A wrong pose here costs an operator a manual
// initialpose; a half-written one taken as a pose would cost more, which is
// what the truncation cases are for.
//
// The power-loss cases (2026-10-23). saveLastPose never writes the file in
// place: <path>.tmp, fsync, rename over <path>, fsync of the directory. A
// power loss can stop that at any step, and at every step the file the next
// boot reads must be a complete one -- the previous pose or the new one, never
// half of either. A test cannot cut the power, so each case builds the state a
// cut at one step leaves on disk (StaleTmp: during the tmp write; CompleteTmp:
// after its fsync, before the rename) or makes a step fail, and checks what the
// reader gets. ReaderNeverSeesAPartialFile is the rename's own guarantee,
// exercised: a reader racing the writer may get the old pose or the new one,
// and nothing else. That is also why there is no .bak beside the file: the
// rename already leaves the previous complete file in every failure case a
// .bak would cover, short of media corruption.
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>

#include "syncai_localizer/localizers/last_pose_store.h"

namespace
{
namespace fs = std::filesystem;

class LastPoseStore : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() /
           ("last_pose_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
            "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  void writeRaw(const fs::path & p, const std::string & text)
  {
    fs::create_directories(p.parent_path());
    std::ofstream(p) << text;
  }

  fs::path dir_;
};

LastPose dp2f()
{
  LastPose p;
  p.map = "dp";
  p.x = 2.0612;
  p.y = 4.6107;
  p.ground_z = 3.1234;
  p.yaw = -0.031416;
  return p;
}

TEST_F(LastPoseStore, RoundTrip)
{
  const std::string path = (dir_ / "last_pose.json").string();
  std::string error;
  ASSERT_TRUE(saveLastPose(path, dp2f(), error)) << error;
  const auto got = loadLastPose(path, error);
  ASSERT_TRUE(got) << error;
  EXPECT_EQ(got->map, "dp");
  EXPECT_NEAR(got->x, 2.0612, 1e-4);
  EXPECT_NEAR(got->y, 4.6107, 1e-4);
  EXPECT_NEAR(got->ground_z, 3.1234, 1e-4);
  EXPECT_NEAR(got->yaw, -0.031416, 1e-6);
  EXPECT_FALSE(got->saved_at.empty());
}

TEST_F(LastPoseStore, CreatesTheDirectoryAndLeavesNoTmp)
{
  // config/state/<robot_id>/ does not exist on a fresh checkout.
  const fs::path path = dir_ / "state" / "robot01" / "last_pose.json";
  std::string error;
  ASSERT_TRUE(saveLastPose(path.string(), dp2f(), error)) << error;
  EXPECT_TRUE(fs::exists(path));
  EXPECT_FALSE(fs::exists(path.string() + ".tmp"));
}

TEST_F(LastPoseStore, OverwriteReplacesTheWholeFile)
{
  const std::string path = (dir_ / "last_pose.json").string();
  std::string error;
  ASSERT_TRUE(saveLastPose(path, dp2f(), error));
  LastPose moved = dp2f();
  moved.map = "conference";
  moved.x = -12.5;
  ASSERT_TRUE(saveLastPose(path, moved, error));
  const auto got = loadLastPose(path, error);
  ASSERT_TRUE(got) << error;
  EXPECT_EQ(got->map, "conference");
  EXPECT_NEAR(got->x, -12.5, 1e-4);
}

TEST_F(LastPoseStore, StaleTmpDoesNotMatter)
{
  // A power loss between the tmp write and the rename leaves <path>.tmp
  // behind; the reader must keep reading the last complete file, and the next
  // save must go through.
  const std::string path = (dir_ / "last_pose.json").string();
  std::string error;
  ASSERT_TRUE(saveLastPose(path, dp2f(), error));
  writeRaw(path + ".tmp", R"({"map": "dp", "x": 9)");
  auto got = loadLastPose(path, error);
  ASSERT_TRUE(got) << error;
  EXPECT_NEAR(got->x, 2.0612, 1e-4);
  LastPose moved = dp2f();
  moved.x = 7.0;
  ASSERT_TRUE(saveLastPose(path, moved, error)) << error;
  got = loadLastPose(path, error);
  ASSERT_TRUE(got);
  EXPECT_NEAR(got->x, 7.0, 1e-4);
}

TEST_F(LastPoseStore, CompleteTmpNotYetRenamed)
{
  // Power lost after the tmp's fsync, before the rename: a complete, valid
  // tmp holding the NEW pose sits beside the old file. The boot must still
  // read the old file (the new one was never committed), and the next save
  // must replace both.
  const std::string path = (dir_ / "last_pose.json").string();
  std::string error;
  ASSERT_TRUE(saveLastPose(path, dp2f(), error));
  writeRaw(
    path + ".tmp",
    R"({"map": "dp", "x": 9.0, "y": 9.0, "ground_z": 0.0, "yaw": 0.0, "saved_at": "x"})");
  auto got = loadLastPose(path, error);
  ASSERT_TRUE(got) << error;
  EXPECT_NEAR(got->x, 2.0612, 1e-4);
  LastPose moved = dp2f();
  moved.x = 7.0;
  ASSERT_TRUE(saveLastPose(path, moved, error)) << error;
  got = loadLastPose(path, error);
  ASSERT_TRUE(got);
  EXPECT_NEAR(got->x, 7.0, 1e-4);
  EXPECT_FALSE(fs::exists(path + ".tmp"));
}

TEST_F(LastPoseStore, FailedWriteLeavesTheOldFile)
{
  // The tmp cannot even be created (a read-only directory stands in for a
  // full or remounted-read-only disk): the save reports the failure and the
  // file on disk is the previous one, untouched.
  if (::geteuid() == 0) GTEST_SKIP() << "root ignores directory permissions";
  const fs::path sub = dir_ / "ro";
  const std::string path = (sub / "last_pose.json").string();
  std::string error;
  ASSERT_TRUE(saveLastPose(path, dp2f(), error));
  ASSERT_EQ(::chmod(sub.c_str(), 0555), 0);
  LastPose moved = dp2f();
  moved.x = 7.0;
  EXPECT_FALSE(saveLastPose(path, moved, error));
  EXPECT_FALSE(error.empty());
  ::chmod(sub.c_str(), 0755);
  const auto got = loadLastPose(path, error);
  ASSERT_TRUE(got) << error;
  EXPECT_NEAR(got->x, 2.0612, 1e-4);
  EXPECT_FALSE(fs::exists(path + ".tmp"));
}

TEST_F(LastPoseStore, FailedRenameCleansUpTheTmp)
{
  // The rename fails (here: a directory sits where the file should be). The
  // save reports it and leaves no tmp behind to be mistaken for anything.
  const fs::path path = dir_ / "last_pose.json";
  fs::create_directories(path);
  std::string error;
  EXPECT_FALSE(saveLastPose(path.string(), dp2f(), error));
  EXPECT_NE(error.find("rename"), std::string::npos) << error;
  EXPECT_FALSE(fs::exists(path.string() + ".tmp"));
}

TEST_F(LastPoseStore, UncreatableDirectoryFailsCleanly)
{
  // config/state/<robot_id>/ cannot be made (a regular file is in the way):
  // a reported failure, not a crash, and nothing written anywhere.
  writeRaw(dir_ / "state", "not a directory");
  std::string error;
  EXPECT_FALSE(
    saveLastPose((dir_ / "state" / "robot01" / "last_pose.json").string(), dp2f(), error));
  EXPECT_FALSE(error.empty());
}

TEST_F(LastPoseStore, ReaderNeverSeesAPartialFile)
{
  // One writer saving as fast as it can, one reader reading as fast as it
  // can. Every read after the first save must parse to a complete pose, and
  // its x must be one the writer wrote, never going backwards. Checked
  // against a copy of saveLastPose that writes the file in place (2026-10-23):
  // it failed on 3 runs of 3, 753 of 47142 reads catching the file truncated
  // or half-written; the tmp + rename version has never failed it.
  const std::string path = (dir_ / "last_pose.json").string();
  std::string error;
  LastPose pose = dp2f();
  pose.x = 0.0;
  ASSERT_TRUE(saveLastPose(path, pose, error)) << error;

  constexpr int kWrites = 300;
  std::atomic<bool> done{false};
  std::atomic<int> reads{0}, bad_reads{0}, backwards{0};
  std::thread reader([&] {
    double last_x = -1.0;
    std::string err;
    while (!done.load()) {
      const auto got = loadLastPose(path, err);
      ++reads;
      if (!got) {
        ++bad_reads;
        continue;
      }
      if (got->x < last_x || got->x != std::floor(got->x) || got->x > kWrites) ++backwards;
      last_x = got->x;
    }
  });
  for (int i = 1; i <= kWrites; ++i) {
    pose.x = i;
    ASSERT_TRUE(saveLastPose(path, pose, error)) << error;
  }
  done = true;
  reader.join();
  EXPECT_GT(reads.load(), 0);
  EXPECT_EQ(bad_reads.load(), 0) << "of " << reads.load() << " reads";
  EXPECT_EQ(backwards.load(), 0);
  const auto got = loadLastPose(path, error);
  ASSERT_TRUE(got);
  EXPECT_EQ(got->x, kWrites);
}

TEST_F(LastPoseStore, MissingFileIsNotAPose)
{
  std::string error;
  EXPECT_FALSE(loadLastPose((dir_ / "nope.json").string(), error));
  EXPECT_FALSE(error.empty());
}

TEST_F(LastPoseStore, TruncatedOrMalformedIsNotAPose)
{
  const fs::path path = dir_ / "last_pose.json";
  std::string error;
  // Cut mid-object, as a write without the tmp + rename would leave it.
  writeRaw(path, R"({"map": "dp", "x": 2.06, "y": 4.6)");
  EXPECT_FALSE(loadLastPose(path.string(), error));
  // Complete JSON, fields missing.
  writeRaw(path, R"({"map": "dp", "x": 2.06, "y": 4.61})");
  EXPECT_FALSE(loadLastPose(path.string(), error));
  // A number written as a string.
  writeRaw(path, R"({"map": "dp", "x": "2.06", "y": 4.61, "ground_z": 0.0, "yaw": 0.0})");
  EXPECT_FALSE(loadLastPose(path.string(), error));
  // No map: a pose for an unknown map must not be applied to any.
  writeRaw(path, R"({"x": 2.06, "y": 4.61, "ground_z": 0.0, "yaw": 0.0})");
  EXPECT_FALSE(loadLastPose(path.string(), error));
  // Not an object.
  writeRaw(path, "[2.06, 4.61]");
  EXPECT_FALSE(loadLastPose(path.string(), error));
  // The old key: value format of the first draft.
  writeRaw(path, "map: dp\nx: 2.06\ny: 4.61\nground_z: 0.0\nyaw: 0.0\n");
  EXPECT_FALSE(loadLastPose(path.string(), error));
  // Empty file.
  writeRaw(path, "");
  EXPECT_FALSE(loadLastPose(path.string(), error));
}

TEST_F(LastPoseStore, WritesJsonTheLaunchCanRead)
{
  // The launch reads it with Python's json module; keep the keys it reads.
  const std::string path = (dir_ / "last_pose.json").string();
  std::string error;
  ASSERT_TRUE(saveLastPose(path, dp2f(), error)) << error;
  std::ifstream in(path);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  for (const char * key :
       {"\"map\": \"dp\"", "\"x\":", "\"y\":", "\"ground_z\":", "\"yaw\":", "\"saved_at\":"}) {
    EXPECT_NE(text.find(key), std::string::npos) << key << " in\n" << text;
  }
}

TEST(MapName, FromPcdPath)
{
  EXPECT_EQ(mapNameFromPcdPath("/home/syncrobotic/robot_ws/map/dp/map.pcd"), "dp");
  EXPECT_EQ(mapNameFromPcdPath("map/ci_smoke_3d/map.pcd"), "ci_smoke_3d");
}
}  // namespace
