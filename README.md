Description
========

Ouster is a library that allows users interact with sensor hardware and recorded sensor data. This is suitable for prototyping, evaluation and other non-safety critical applications using Python or C++. The C++ library has a wide range of use cases for implementing and intereacting with sensors while the Python library supports frame-based access to lidar data as numpy datatypes and a responsive visualizer for pcap and sensor.


What you can achieve with the Library
--------

 - Sensor configuration
 - Recording and reading data in pcap format
 - Reading and buffering sensor UDP data streams reliably
 - Conversion of raw data to range/signal/near_ir/reflectivity images (destaggering)
 - Efficient projection of range measurements to Cartesian (x, y, z) corrdinates
 - Visualization of multi-beam flash lidar data
 - Frame-based access to lidar data as numpy datatypes
 - A responsive visualizer utility for pcap and sensor


Ouster Python Library Supported Platforms 
----------------------------------------

Ouster Python Library Supported Platforms

Pre-built binaries are provided on PyPI for the following platforms:

    Most glibc-based Linux distributions on x86_64 and ARM64 platforms (manylinux2010_x86_64, manylinux2014_aarch64)
    macOS >= 10.13 on x86_64 platforms (macosx_10_13_x86_64)
    macOS >= 11.0 on Apple M1 for Python >= 3.8 (macosx_11_0_arm64)
    Windows 10 on x86_64 platforms (win_amd64)

Building from source is supported on:

    Ubuntu 18.04, 20.04, and Debian 10 (x86-64, aarch64)
    macOS >= 10.13 (x86-64), >= 11.0 (arm64)
    Windows 10 (x86-64)

Usage syntax and Sample
----------------------

Ouster library supports two languages, Python and C++. The complete sample for all the implementation will be provided, the usage snytax and sample aims to explain some of the critical aspects of the example to facilitate easy use. 


Sensor Configuration using C++
------------------------------

The first step is to get the sensor starting. By getting the sensor started, it works with the current config on the hostname. 

```cpp
const std::string sensor_hostname = argv[1];

    // 1. Get the current config on the sensor
    std::cerr << "1. Get original config of sensor... ";

    //! [doc-stag-cpp-get-config]
    sensor::sensor_config original_config;
    if (!sensor::get_config(sensor_hostname, original_config)) {
        std::cerr << "..error: could not connect to sensor!" << std::endl;
        return EXIT_FAILURE;
    }
    //! [doc-etag-cpp-get-config]
    std::cerr << "success! Got original config\nOriginal config of sensor:\n"
              << to_string(original_config) << std::endl;
```

The next step is to empty the current sensor configuration and set a few configuration parameters. This allows you to modify the current config

```cpp
std::cerr << "\n2. Make new config and set sensor to it... ";
    //! [doc-stag-cpp-make-config]
    sensor::sensor_config config;
    config.azimuth_window = std::make_pair<int>(90000, 270000);
    config.ld_mode = sensor::lidar_mode::MODE_512x10;

    // If relevant, use config_flag to set udp dest automatically
    uint8_t config_flags = 0;
    const bool udp_dest_auto = true;  // whether or not to use auto destination
    const bool persist =
        false;  // whether or not we will persist the settings on the sensor

    if (udp_dest_auto) config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
    if (persist) config_flags |= ouster::sensor::CONFIG_PERSIST;
    //! [doc-etag-cpp-make-config]

    if (!sensor::set_config(sensor_hostname, config, config_flags)) {
        std::cerr << "..error: could not connect to sensor" << std::endl;
        return EXIT_FAILURE;
    }
    std::cerr << "..success! Updated sensor to new config" << std::endl;
```

These two steps suffices but you can confirm the config from the sensor after it has been updated. This is just to make sure you have the correct config

```cpp
 std::cerr << "\n3. Get back updated sensor config... ";
    sensor::sensor_config new_config;
    if (!sensor::get_config(sensor_hostname, new_config)) {
        std::cerr << "..error: could not connect to sensor" << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cerr << "..success! Got updated config" << std::endl;
    }
 ```
 
 You can also confirm that only what you wanted to change changed
 
 ```cpp
 assert(original_config != new_config);
    assert(new_config.azimuth_window == config.azimuth_window);
    assert(new_config.ld_mode == config.ld_mode);

    std::cerr << "Updated config: \n" << to_string(new_config) << std::endl;
 ```
 
 ### Lidar Scan
 
 You can also create a LidarScan by providing a lidar profile, avilable through the sensor_info. The complete example is demostrated in the linder_scan_example file.
 
 ```cpp
 auto profile_scan = ouster::LidarScan(w, h, info.format.udp_profile_lidar);
    //! [doc-etag-lidarscan-profile-constructor]

    // You might have a dual returns sensor, in which case your profile will
    // reflect that it is a dual return profile:
    //! [doc-stag-lidarscan-dual-profile-constructor]
    auto dual_returns_scan = ouster::LidarScan(
        w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
    //! [doc-etag-lidarscan-dual-profile-constructor]

    //! [doc-stag-lidarscan-reduced-slots]
    // Finally, you can construct by specifying fields directly
    static const std::array<std::pair<ChanField, ChanFieldType>, 2>
        reduced_slots{{{ChanField::RANGE, ChanFieldType::UINT32},
                       {ChanField::REFLECTIVITY, ChanFieldType::UINT8}}};
    auto reduced_fields_scan =
        ouster::LidarScan(w, h, reduced_slots.begin(), reduced_slots.end());
 ```
 
 ### 2D Representations and 3D representations
 
 Working with 2D representations and 3D representations allows users to reshape the X,Y,Z coordinates into a 2D and also adjust XYZLut with external matrix. 
 
 ```cpp
 img_t<double> get_x_in_image_form(const LidarScan& scan, bool destaggered,
                                  const sensor::sensor_info& info) {
    // For convenience, save w and h to variables
    const size_t w = info.format.columns_per_frame;
    const size_t h = info.format.pixels_per_column;

    // Get the XYZ in ouster::Points (n x 3 Eigen array) form
    XYZLut lut = make_xyz_lut(info);
    auto cloud = cartesian(scan.field(sensor::ChanField::RANGE), lut);

    // Access x and reshape as needed
    // Note that the values in cloud.col(0) are ordered
    auto x = Eigen::Map<const img_t<double>>(cloud.col(0).data(), h, w);
    auto x_destaggered = destagger<double>(x, info.format.pixel_shift_by_row);

    // Apply destagger if desired
    if (!destaggered) return x;
    return x_destaggered;
}
 ```
 
Adjust XYZLut with external matrix

```cpp
 auto lut_extrinsics = make_xyz_lut(
        w, h, sensor::range_unit, info.lidar_origin_to_beam_origin_mm,
        transformation, info.beam_azimuth_angles, info.beam_altitude_angles);

    std::cerr << "Calculating 3d Points of with special transform provided.."
              << std::endl;
    auto cloud_adjusted = cartesian(range, lut_extrinsics);
    //! [doc-etag-extrinsics-to-xyzlut]

    std::cerr << "And now the 2000th point in the transformed point cloud... ("
              << cloud_adjusted(2000, 0) << ", " << cloud_adjusted(2000, 1)
              << ", " << cloud_adjusted(2000, 2) << ")" << std::endl;
```

Sensor Configuration using Python
---------------------------------

Configuring sensor requires the user to configure two different ports. These ports are the operating mode and lidar mode respectively. The argument to be passed for this function is the hostname which is the hostname of the sensor.

```python
# create empty config
    config = client.SensorConfig()

    # set the values that you need: see sensor documentation for param meanings
    config.operating_mode = client.OperatingMode.OPERATING_NORMAL
    config.lidar_mode = client.LidarMode.MODE_1024x10
    config.udp_port_lidar = 7502
    config.udp_port_imu = 7503

    # set the config on sensor, using appropriate flags
    client.set_config(hostname, config, persist=True, udp_dest_auto=True)
    # [doc-etag-configure]

    # if you like, you can view the entire set of parameters
    config = client.get_config(hostname)
    print(f"sensor config of {hostname}:\n{config}")
```

### Fetch metadata from a sensor 

Accurately reconstructing point clouds from a sensor data stream requires access to sensor calibration and per-run configuration like the operating mode and azimuth window. The client API makes it easy to read metadata and write it to disk for use with recorded data streams. he argument to be passed for this function is the hostname which is the hostname of the sensor.

```python
with closing(client.Sensor(hostname)) as source:
        # print some useful info from
        print("Retrieved metadata:")
        print(f"  serial no:        {source.metadata.sn}")
        print(f"  firmware version: {source.metadata.fw_rev}")
        print(f"  product line:     {source.metadata.prod_line}")
        print(f"  lidar mode:       {source.metadata.mode}")
        print(f"Writing to: {hostname}.json")

        # write metadata to disk
        source.write_metadata(f"{hostname}.json")
```

### Record data from live sensor to pcap file

Note that pcap files recorded this way only preserve the UDP data stream and not networking information, unlike capturing packets directly from a network
interface with tools like tcpdump or wireshark.

The Arguments 

hostname: hostname of the sensor
lidar_port: UDP port to listen on for lidar data
imu_port: UDP port to listen on for imu data
n_seconds: max seconds of time to record. (Ctrl-Z correctly closes streams)

```python
import ouster.pcap as pcap
    from datetime import datetime

    # [doc-stag-pcap-record]
    from more_itertools import time_limited
    # connect to sensor and record lidar/imu packets
    with closing(client.Sensor(hostname, lidar_port, imu_port,
                               buf_size=640)) as source:

        # make a descriptive filename for metadata/pcap files
        time_part = datetime.now().strftime("%Y%m%d_%H%M%S")
        meta = source.metadata
        fname_base = f"{meta.prod_line}_{meta.sn}_{meta.mode}_{time_part}"

        print(f"Saving sensor metadata to: {fname_base}.json")
        source.write_metadata(f"{fname_base}.json")

        print(f"Writing to: {fname_base}.pcap (Ctrl-C to stop early)")
        source_it = time_limited(n_seconds, source)
        n_packets = pcap.record(source_it, f"{fname_base}.pcap")

        print(f"Captured {n_packets} packets")
```
### Display range from a single scan as 3D points

This function allows users to display range from a single scan as 3D objets. The arguments are hostname of the sensor and UDP port to listen on for lidar data. 

```python
import matplotlib.pyplot as plt  # type: ignore

    # get single scan
    metadata, sample = client.Scans.sample(hostname, 1, lidar_port)
    scan = next(sample)[0]

    # set up figure
    plt.figure()
    ax = plt.axes(projection='3d')
    r = 3
    ax.set_xlim3d([-r, r])
    ax.set_ylim3d([-r, r])
    ax.set_zlim3d([-r, r])

    plt.title("3D Points from {}".format(hostname))

    # [doc-stag-plot-xyz-points]
    # transform data to 3d points
    xyzlut = client.XYZLut(metadata)
    xyz = xyzlut(scan.field(client.ChanField.RANGE))
    # [doc-etag-plot-xyz-points]

    # graph xyz
    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax.scatter(x, y, z, c=z / max(z), s=0.2)
    plt.show()
```

