#!/usr/bin/env python3
"""
Colorize a Voxel-SLAM point cloud using RGB and thermal camera images.

For each LiDAR point, projects into the nearest-time camera image using
known LiDAR-camera extrinsics and fisheye intrinsics (equidistant model).

Uses cv2.fisheye.projectPoints for correct equidistant projection.

Usage:
    colorize-pointcloud.py <pcd_dir> <pose_file> <bag_dir> <output.laz>
        [--voxel 0.05] [--max-range 30] [--max-scans N]
"""
import sys, os, time, re, json
import numpy as np
import cv2
from pathlib import Path
from scipy.spatial.transform import Rotation

try:
    import laspy
except ImportError:
    sys.exit("pip install laspy lazrs")
try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError:
    sys.exit("pip install mcap mcap-ros2-support")


def load_calibration(calib_dir, cam_name):
    calib_path = Path(calib_dir) / cam_name / 'calibration.json'
    ext_path = Path(calib_dir) / cam_name / 'dvlc_extrinsic.json'
    intrinsics = extrinsics = None
    if calib_path.exists():
        d = json.load(open(calib_path))
        intrinsics = {'K': np.array(d['K']), 'D': np.array(d['D']),
                      'w': d['image_size'][0], 'h': d['image_size'][1],
                      'model': d.get('model', 'equidistant')}
    if ext_path.exists():
        d = json.load(open(ext_path))
        tlc = d['results']['T_lidar_camera']
        t_lc = np.array(tlc[:3])
        R_lc = Rotation.from_quat(tlc[3:]).as_matrix()
        R_cl = R_lc.T; t_cl = -R_cl @ t_lc
        extrinsics = {'R_cl': R_cl, 't_cl': t_cl}
    return intrinsics, extrinsics


def project_fisheye(pts_lidar, R_cl, t_cl, K, D, w, h):
    pts_cam = (R_cl @ pts_lidar.T).T + t_cl
    in_front = pts_cam[:, 2] > 0.1
    rvec = cv2.Rodrigues(R_cl)[0]
    tvec = t_cl.reshape(3, 1)
    D4 = np.array(D[:4], dtype=np.float64).reshape(4, 1)
    pts_2d, _ = cv2.fisheye.projectPoints(
        pts_lidar.reshape(-1, 1, 3).astype(np.float64), rvec, tvec,
        K.astype(np.float64), D4)
    px = pts_2d.reshape(-1, 2)
    valid = in_front & (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h) & np.isfinite(px).all(axis=1)
    return px, valid


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd_dir'); parser.add_argument('pose_file')
    parser.add_argument('bag_dir'); parser.add_argument('output')
    parser.add_argument('--voxel', type=float, default=0.05)
    parser.add_argument('--max-range', type=float, default=30.0)
    parser.add_argument('--max-scans', type=int, default=0)
    parser.add_argument('--calib-dir', default='/opt/bess/config/cameras/calibration')
    args = parser.parse_args()

    pcd_dir = Path(args.pcd_dir); bag_dir = Path(args.bag_dir)

    # Calibrations
    print("Loading calibrations...", flush=True)
    cameras = []
    for name in ['camera2', 'camera3']:
        intr, ext = load_calibration(args.calib_dir, name)
        if intr and ext:
            cameras.append({'name': name, **intr, **ext})
            print(f"  {name}: {intr['model']} {intr['w']}x{intr['h']}")

    # Poses
    poses = []
    with open(args.pose_file) as f:
        for line in f:
            v = [float(x) for x in line.strip().split()]
            poses.append({'ts': v[0], 'R': Rotation.from_quat([v[4],v[5],v[6],v[7]]).as_matrix(),
                          'p': np.array([v[1],v[2],v[3]])})
    pose_ts = np.array([p['ts'] for p in poses])
    print(f"  {len(poses)} poses", flush=True)

    # Pre-load camera images
    print("Loading camera images...", flush=True)
    cam_topics = {'/camera2/camera_driver/image_masked/compressed': 'camera2',
                  '/camera3/camera_driver/image_masked/compressed': 'camera3'}
    images = {n: [] for n in ['camera2', 'camera3']}

    mcaps = sorted(bag_dir.glob('*.mcap'),
                   key=lambda f: int(re.search(r'_(\d+)\.mcap$', f.name).group(1))
                   if re.search(r'_(\d+)\.mcap$', f.name) else 0)
    for mp in mcaps[:4]:
        with open(mp, 'rb') as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, ch, _, msg in reader.iter_decoded_messages(topics=list(cam_topics.keys())):
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if bgr is not None:
                    images[cam_topics[ch.topic]].append((ts, bgr))

    for n in list(images.keys()):
        images[n].sort(key=lambda x: x[0])
        images[f'{n}_ts'] = np.array([x[0] for x in images[n]]) if images[n] else np.array([])
        print(f"  {n}: {len(images[n])} images")

    # T_lidar_imu (to convert body→lidar for projection)
    R_li = np.array([[-1,0,0],[0,-1,0],[0,0,1]], dtype=np.float64)
    t_li = np.array([-0.015, 0.0, -0.20])

    # Process PCDs
    print(f"\nColorizing...", flush=True)
    pcds = sorted(pcd_dir.glob('*.pcd'), key=lambda f: int(f.stem))
    all_xyz, all_rgb = [], []
    n_total, n_colored = 0, 0
    t0 = time.time()

    for i, pp in enumerate(pcds):
        if args.max_scans and i >= args.max_scans: break
        if i >= len(poses): break
        with open(pp, 'rb') as f:
            n_pts = 0
            while True:
                l = f.readline().decode('ascii', errors='ignore').strip()
                if l.startswith('POINTS'): n_pts = int(l.split()[1])
                if l.startswith('DATA binary'): break
            data = np.frombuffer(f.read(n_pts*16), dtype=np.float32).reshape(-1, 4)

        v = np.isfinite(data[:,:3]).all(axis=1) & ((data[:,0]!=0)|(data[:,1]!=0)|(data[:,2]!=0))
        v &= (data[:,0]**2+data[:,1]**2+data[:,2]**2) <= args.max_range**2
        if v.sum() < 100: continue

        pcd_body = data[v, :3]
        xyz_w = (poses[i]['R'] @ pcd_body.astype(np.float64).T).T + poses[i]['p']

        # Body→LiDAR frame for projection
        pcd_lidar = (R_li.T @ (pcd_body.astype(np.float64) - t_li).T).T

        rgb = np.zeros((len(pcd_body), 3), dtype=np.uint8)
        colored = np.zeros(len(pcd_body), dtype=bool)

        for cam in cameras:
            if len(images[cam['name']]) == 0: continue
            cts = images[f"{cam['name']}_ts"]
            ii = np.argmin(np.abs(cts - poses[i]['ts']))
            if abs(cts[ii] - poses[i]['ts']) > 0.5: continue
            bgr = images[cam['name']][ii][1]

            px, vm = project_fisheye(pcd_lidar, cam['R_cl'], cam['t_cl'],
                                     cam['K'], cam['D'], cam['w'], cam['h'])
            new = vm & ~colored
            if new.any():
                u = np.clip(px[new,0].astype(int), 0, bgr.shape[1]-1)
                vv = np.clip(px[new,1].astype(int), 0, bgr.shape[0]-1)
                rgb[new] = bgr[vv, u, ::-1]  # BGR→RGB
                colored |= new

        all_xyz.append(xyz_w.astype(np.float32))
        all_rgb.append(rgb)
        n_total += len(pcd_body); n_colored += colored.sum()

        if (i+1) % 200 == 0:
            pts = sum(len(a) for a in all_xyz)
            print(f"  {i+1} scans, {pts/1e6:.1f}M pts, {n_colored/n_total*100:.0f}% colored, {time.time()-t0:.0f}s", flush=True)

    xyz = np.concatenate(all_xyz).astype(np.float64)
    rgb = np.concatenate(all_rgb)

    # Voxel filter
    vk = np.floor(xyz/args.voxel).astype(np.int32)
    vp = np.ascontiguousarray(vk).view(np.dtype((np.void,12))).ravel()
    _, ui = np.unique(vp, return_index=True)
    print(f"Voxel: {len(xyz)/1e6:.1f}M → {len(ui)/1e6:.1f}M", flush=True)

    xyz = xyz[ui]; rgb = rgb[ui]

    # Write LAZ with RGB (point format 2)
    header = laspy.LasHeader(point_format=2, version="1.4")
    header.offsets = np.min(xyz, axis=0); header.scales = np.array([0.001]*3)
    las = laspy.LasData(header)
    las.x = xyz[:,0]; las.y = xyz[:,1]; las.z = xyz[:,2]
    las.red = rgb[:,0].astype(np.uint16) * 256
    las.green = rgb[:,1].astype(np.uint16) * 256
    las.blue = rgb[:,2].astype(np.uint16) * 256
    las.write(args.output)
    print(f"\nDone: {args.output} ({os.path.getsize(args.output)/1e6:.0f}MB, {len(ui):,} pts, {n_colored/n_total*100:.0f}% colored)")


if __name__ == '__main__':
    main()
