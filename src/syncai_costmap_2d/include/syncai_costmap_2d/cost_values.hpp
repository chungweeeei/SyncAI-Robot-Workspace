#ifndef SYNCAI_NAV2_COSTMAP_2D__COST_VALUES_HPP_
#define SYNCAI_NAV2_COSTMAP_2D__COST_VALUES_HPP_

namespace syncai_costmap_2d
{

enum class CombinationMethod : int {
  /**
    * CombinationMethod enumerate class 是用來定義 在costmap中「每一層 layer 要如何把自己的值合併到master costmap」的策略
    * Overwrite - 直接覆蓋 master 的值（但 NO_INFORMATION 不寫入）
    *           - Static Layer: 靜態地圖是ground truth，要直接蓋掉
    * Max - 取 master 和 layer 中比較大的那個。 NO_INFORMATION 也會被「比較大的有效值」蓋掉
    *     - ObstacleLayer — 偵測到障礙就要保留（致命值最大）
    * MaxWithoutUnknownOverwrite - 同 Max，但保留 master 原本的 NO_INFORMATION（不讓 unknown 被蓋成 known）
    *                            - 想保留「未知就是未知」語意的場合，例如某些 inflation 用法
    * Max是costmap預設的合併方式。
    */

  Overwrite = 0,
  Max = 1,
  MaxWithoutUnknownOverwrite = 2
};

static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_NAV2_COSTMAP_2D__COST_VALUES_HPP_