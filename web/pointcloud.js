import { lidarPoints } from "./sensors.js";

function returns(reading) {
  const points = lidarPoints(reading);
  let next = 0;
  return reading.rangesM.flatMap((range, index) => {
    if (range === null) return [];
    const channel = Math.floor(index / reading.azimuthCount);
    return [
      {
        point: points[next++],
        channel,
        range,
        azimuth:
          reading.angleStartRad +
          (index % reading.azimuthCount) * reading.angleStepRad,
        elevation:
          reading.elevationStartRad + channel * reading.elevationStepRad,
      },
    ];
  });
}

export function exportLidarCsv(reading) {
  return (
    [
      "time_s,channel,azimuth_rad,elevation_rad,range_m,north_m,east_m,down_m",
      ...returns(reading).map(({ point, channel, range, azimuth, elevation }) =>
        [reading.time, channel, azimuth, elevation, range, ...point].join(","),
      ),
    ].join("\n") + "\n"
  );
}

// PCD 0.7: https://pointclouds.org/documentation/tutorials/pcd_file_format.html
export function exportLidarPcd(reading) {
  const points = returns(reading);
  const pose =
    reading.positionNed === null
      ? [0, 0, 0, 1, 0, 0, 0]
      : [...reading.positionNed, ...reading.quaternion];
  return (
    [
      "# .PCD v0.7 - Point Cloud Data file format",
      "# frame NED: x=north, y=east, z=down; units meters",
      `# acquisition_time_s ${reading.time}`,
      `# acquisition_pose ${reading.positionNed === null ? "unavailable; VIEWPOINT is the PCD default" : "recorded in NED, quaternion wxyz"}`,
      "VERSION 0.7",
      "FIELDS x y z ring range",
      "SIZE 8 8 8 2 8",
      "TYPE F F F U F",
      "COUNT 1 1 1 1 1",
      `WIDTH ${points.length}`,
      "HEIGHT 1",
      `VIEWPOINT ${pose.join(" ")}`,
      `POINTS ${points.length}`,
      "DATA ascii",
      ...points.map(({ point, channel, range }) =>
        [...point, channel, range].join(" "),
      ),
    ].join("\n") + "\n"
  );
}
