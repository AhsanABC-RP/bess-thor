#!/usr/bin/env python3
"""Extract single LiDAR scans matched to camera images for dvlc.

Reads a ROS2 bag, picks N evenly-spaced point cloud messages, finds the
closest camera image in time, and saves them in dvlc preprocess format:
  {dst}/frame_NNNN.ply   - single LiDAR scan
  {dst}/frame_NNNN.png   - matched camera image
  {dst}/frame_NNNN_lidar_intensities.png - LiDAR intensity image
  {dst}/calib.json        - camera intrinsics + metadata
"""
import sys
import json
import struct
import argparse
import numpy as np
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', help='Path to ROS2 bag directory')
    parser.add_argument('dst_path', help='Output directory')
    parser.add_argument('--image_topic', default='/thermal2/camera_driver/image_raw')
    parser.add_argument('--points_topic', default='/ouster/points')
    parser.add_argument('--n_frames', type=int, default=5)
    parser.add_argument('--camera_intrinsics', default='334.78419,332.67743,317.07016,234.32726',
                        help='fx,fy,cx,cy of raw (distorted) camera')
    parser.add_argument('--camera_distortion', default='-0.164539,0.026521,-0.004782,-0.004943,0.0')
    parser.add_argument('--distortion_model', default='plumb_bob',
                        help='plumb_bob or equidistant (fisheye)')
    parser.add_argument('--rectified_intrinsics', default='275.82727,300.28354,305.91104,228.86745',
                        help='fx,fy,cx,cy after undistortion (from projection matrix)')
    parser.add_argument('--intensity_channel', default='reflectivity')
    args = parser.parse_args()

    # Import rosbag2 reader
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2, Image

    dst = Path(args.dst_path)
    dst.mkdir(parents=True, exist_ok=True)

    # Open bag
    reader = SequentialReader()
    storage_opts = StorageOptions(uri=args.bag_path, storage_id='')
    converter_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_opts, converter_opts)

    # First pass: collect all timestamps for points and images
    print("Scanning bag for timestamps...")
    points_times = []
    image_times = []
    points_data = {}
    image_data = {}

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        t_sec = timestamp / 1e9
        if topic == args.points_topic:
            points_times.append((t_sec, data))
        elif topic == args.image_topic:
            image_times.append((t_sec, data))

    print(f"Found {len(points_times)} point clouds, {len(image_times)} images")

    if not points_times or not image_times:
        print("ERROR: No data found")
        sys.exit(1)

    # Pick N evenly spaced point clouds
    n = min(args.n_frames, len(points_times))
    indices = np.linspace(0, len(points_times) - 1, n, dtype=int)

    bag_names = []
    for i, idx in enumerate(indices):
        pc_time, pc_data = points_times[idx]

        # Find closest image
        img_times_arr = np.array([t for t, _ in image_times])
        closest_idx = np.argmin(np.abs(img_times_arr - pc_time))
        img_time, img_data = image_times[closest_idx]
        dt = abs(pc_time - img_time)

        print(f"Frame {i}: pc_t={pc_time:.3f} img_t={img_time:.3f} dt={dt*1000:.1f}ms")

        # Deserialize
        pc_msg = deserialize_message(pc_data, PointCloud2)
        img_msg = deserialize_message(img_data, Image)

        # Extract points from PointCloud2
        xyz, intensities = pointcloud2_to_xyz(pc_msg, args.intensity_channel)
        print(f"  Points: {len(xyz)}, intensity range: {intensities.min():.0f}-{intensities.max():.0f}")

        # Save PLY
        frame_name = f"frame_{i:04d}"
        bag_names.append(frame_name)
        save_ply(dst / f"{frame_name}.ply", xyz, intensities)

        # Save camera image (undistorted)
        K_raw = [float(x) for x in args.camera_intrinsics.split(',')]
        dist = [float(x) for x in args.camera_distortion.split(',')]
        K_rect = [float(x) for x in args.rectified_intrinsics.split(',')]
        save_image(dst / f"{frame_name}.png", img_msg, K_raw, dist, K_rect,
                   args.distortion_model)

        # Save LiDAR intensity image (dvlc uses this for feature matching)
        save_lidar_intensity_image(dst / f"{frame_name}_lidar_intensities.png",
                                   pc_msg, intensities)

    # Save calib.json — use rectified intrinsics (images are undistorted)
    K_rect = [float(x) for x in args.rectified_intrinsics.split(',')]
    calib = {
        "camera": {
            "camera_model": "plumb_bob",
            "intrinsics": K_rect,
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]
        },
        "meta": {
            "bag_names": bag_names,
            "image_topic": args.image_topic,
            "points_topic": args.points_topic,
            "intensity_channel": args.intensity_channel,
            "accumulate_scans": 1
        }
    }
    with open(dst / 'calib.json', 'w') as f:
        json.dump(calib, f, indent=2)

    print(f"\nDone! {n} frames saved to {dst}")


def pointcloud2_to_xyz(msg, intensity_field='reflectivity'):
    """Extract xyz + intensity from PointCloud2 message."""
    # Parse fields
    fields = {f.name: (f.offset, f.datatype, f.count) for f in msg.fields}
    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    n_points = msg.width * msg.height

    # ROS2 PointField datatype mapping
    dtype_map = {
        1: ('i1', 1), 2: ('u1', 1), 3: ('i2', 2), 4: ('u2', 2),
        5: ('i4', 4), 6: ('u4', 4), 7: ('f4', 4), 8: ('f8', 8),
    }

    def read_field(name):
        offset, datatype, count = fields[name]
        dt_str, dt_size = dtype_map[datatype]
        out = np.zeros(n_points, dtype=np.float64)
        for i in range(n_points):
            start = i * point_step + offset
            out[i] = np.frombuffer(data[start:start+dt_size], dtype=dt_str)[0]
        return out

    # Use structured array with explicit offsets (handles gaps between fields)
    dt_fields = []
    for f in msg.fields:
        dt_str, _ = dtype_map[f.datatype]
        dt_fields.append((f.name, dt_str, f.offset))
    dt = np.dtype({'names': [n for n,_,_ in dt_fields],
                   'formats': [f for _,f,_ in dt_fields],
                   'offsets': [o for _,_,o in dt_fields],
                   'itemsize': point_step})
    points = np.frombuffer(msg.data, dtype=dt, count=n_points)

    xyz = np.stack([points['x'].astype(np.float64),
                    points['y'].astype(np.float64),
                    points['z'].astype(np.float64)], axis=1)

    if intensity_field in points.dtype.names:
        intensities = points[intensity_field].astype(np.float64)
    elif 'intensity' in points.dtype.names:
        intensities = points['intensity'].astype(np.float64)
    else:
        intensities = np.zeros(n_points)

    # Filter out zero/nan points
    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)
    return xyz[valid], intensities[valid]


def save_ply(path, xyz, intensities):
    """Save point cloud as binary PLY."""
    n = len(xyz)
    header = f"""ply
format binary_little_endian 1.0
element vertex {n}
property float x
property float y
property float z
property float intensity
end_header
"""
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        data = np.zeros(n, dtype=[('x','f4'),('y','f4'),('z','f4'),('r','f4')])
        data['x'] = xyz[:, 0].astype(np.float32)
        data['y'] = xyz[:, 1].astype(np.float32)
        data['z'] = xyz[:, 2].astype(np.float32)
        data['r'] = intensities.astype(np.float32)
        f.write(data.tobytes())


def save_image(path, msg, K_raw=None, dist_coeffs=None, K_rect=None,
               distortion_model='plumb_bob'):
    """Save sensor_msgs/Image as undistorted PNG."""
    import cv2
    h, w = msg.height, msg.width
    enc = msg.encoding

    if enc in ('mono16', '16UC1'):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        # Normalize to 8-bit B&W with CLAHE for high-contrast features
        v_min, v_max = np.percentile(raw, [1, 99])
        norm = np.clip((raw.astype(np.float32) - v_min) / max(v_max - v_min, 1) * 255, 0, 255).astype(np.uint8)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        img = clahe.apply(norm)
    elif enc in ('mono8', '8UC1'):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
    elif enc in ('bayer_rggb8', 'bayer_rggb16', 'bayer_gbrg8', 'bayer_grbg8', 'bayer_bggr8'):
        # Bayer pattern — debayer to BGR
        bayer_map = {
            'bayer_rggb8': cv2.COLOR_BayerRG2BGR, 'bayer_gbrg8': cv2.COLOR_BayerGB2BGR,
            'bayer_grbg8': cv2.COLOR_BayerGR2BGR, 'bayer_bggr8': cv2.COLOR_BayerBG2BGR,
        }
        if '16' in enc:
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            raw8 = (raw >> 8).astype(np.uint8)
            img = cv2.cvtColor(raw8, cv2.COLOR_BayerRG2BGR)
        else:
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            img = cv2.cvtColor(raw, bayer_map.get(enc, cv2.COLOR_BayerRG2BGR))
    elif enc == 'bgr8':
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc == 'rgb8':
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
    else:
        bpp = msg.step // w
        if bpp == 2:
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            img = (raw.astype(np.float32) / max(raw.max(), 1) * 255).astype(np.uint8)
        else:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)

    # Auto-brighten dark images (RGB)
    if img.ndim == 3 and img.shape[2] == 3:
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_mean = lab[:, :, 0].mean()
        if l_mean < 80:  # dark image
            clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # Undistort using raw intrinsics → rectified intrinsics
    if K_raw is not None and dist_coeffs is not None:
        K_mat = np.array([[K_raw[0], 0, K_raw[2]],
                          [0, K_raw[1], K_raw[3]],
                          [0, 0, 1]], dtype=np.float64)
        dist_arr = np.array(dist_coeffs, dtype=np.float64)
        if K_rect is not None:
            new_K = np.array([[K_rect[0], 0, K_rect[2]],
                              [0, K_rect[1], K_rect[3]],
                              [0, 0, 1]], dtype=np.float64)
        else:
            new_K = K_mat

        if distortion_model == 'equidistant':
            # Fisheye model — use cv2.fisheye
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K_mat, dist_arr, np.eye(3), new_K,
                (img.shape[1], img.shape[0]), cv2.CV_16SC2)
            img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        else:
            img = cv2.undistort(img, K_mat, dist_arr, None, new_K)

    cv2.imwrite(str(path), img)


def save_lidar_intensity_image(path, pc_msg, intensities):
    """Create a 2D intensity image from the Ouster point cloud.
    Ouster PointCloud2 is organized as (height=128, width=1024)."""
    import cv2
    h, w = pc_msg.height, pc_msg.width
    if h > 1 and w > 1:
        # Organized point cloud — reshape directly
        # But intensities were filtered, need full array
        fields = {f.name: (f.offset, f.datatype) for f in pc_msg.fields}
        point_step = pc_msg.point_step
        n = h * w

        dt_map = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
        dt = np.dtype({'names': [f.name for f in pc_msg.fields],
                       'formats': [dt_map.get(f.datatype, 'f4') for f in pc_msg.fields],
                       'offsets': [f.offset for f in pc_msg.fields],
                       'itemsize': point_step})
        points = np.frombuffer(pc_msg.data, dtype=dt, count=n)

        for name in ('reflectivity', 'intensity', 'signal'):
            if name in points.dtype.names:
                int_img = points[name].reshape(h, w).astype(np.float32)
                break
        else:
            int_img = np.zeros((h, w), dtype=np.float32)

        # Normalize
        v_min, v_max = np.percentile(int_img[int_img > 0], [2, 98]) if (int_img > 0).any() else (0, 1)
        norm = np.clip((int_img - v_min) / max(v_max - v_min, 1) * 255, 0, 255).astype(np.uint8)
        cv2.imwrite(str(path), norm)
    else:
        # Unorganized — skip
        cv2.imwrite(str(path), np.zeros((128, 1024), dtype=np.uint8))


if __name__ == '__main__':
    main()
