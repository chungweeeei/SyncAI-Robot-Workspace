"""Pack road_free.ply / occupied.ply / poses.txt into one self-contained three.js HTML viewer."""
import base64, json, sys
import numpy as np

d, tpl, out = sys.argv[1], sys.argv[2], sys.argv[3]

def read_ply(p):
    with open(p, "rb") as f:
        n = 0
        while True:
            l = f.readline().decode().strip()
            if l.startswith("element vertex"): n = int(l.split()[2])
            if l == "end_header": break
        return np.frombuffer(f.read(n * 12), dtype="<f4").reshape(n, 3).astype(np.float64)

road = read_ply(f"{d}/road_free.ply")
occ = read_ply(f"{d}/occupied.ply")
occ = occ[np.unique(np.floor(occ / 0.1).astype(np.int64), axis=0, return_index=True)[1]]  # 0.1 m for walls
traj = np.loadtxt(f"{d}/poses.txt", usecols=(1, 2, 3))

center = np.round(np.vstack([road, occ]).mean(0)[:2], 2)
def pack(p):  # int16 centimetres relative to the centre, little endian
    q = np.round((p - [center[0], center[1], 0]) * 100).astype(np.int64)
    assert np.abs(q).max() < 32767, "cloud too large for int16 cm"
    return base64.b64encode(q.astype("<i2").tobytes()).decode()

meta = dict(
    name=sys.argv[4] if len(sys.argv) > 4 else d.rstrip("/").split("/")[-1], center=center.tolist(), voxel=0.05,
    road_n=len(road), occ_n=len(occ), traj_n=len(traj),
    road_area=round(len(road) * 0.05 * 0.05, 1),
    zmin=float(np.percentile(road[:, 2], 1)), zmax=float(np.percentile(road[:, 2], 99)),
)
html = open(tpl).read()
html = html.replace("__META__", json.dumps(meta)).replace("__ROAD__", pack(road)) \
           .replace("__OCC__", pack(occ)).replace("__TRAJ__", pack(traj))
open(out, "w").write(html)
print(meta, f"{len(html)/1e6:.1f} MB")
