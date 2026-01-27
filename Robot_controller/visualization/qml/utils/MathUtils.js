// Math utility functions for QML
// These are used for coordinate transformations and quaternion operations

.pragma library

// Convert euler angles (in degrees, ZYX convention) to quaternion
function eulerToQuaternion(euler) {
    var roll = euler.x * Math.PI / 180.0;   // X rotation
    var pitch = euler.y * Math.PI / 180.0;  // Y rotation
    var yaw = euler.z * Math.PI / 180.0;    // Z rotation

    var cr = Math.cos(roll * 0.5);
    var sr = Math.sin(roll * 0.5);
    var cp = Math.cos(pitch * 0.5);
    var sp = Math.sin(pitch * 0.5);
    var cy = Math.cos(yaw * 0.5);
    var sy = Math.sin(yaw * 0.5);

    var w = cr * cp * cy + sr * sp * sy;
    var x = sr * cp * cy - cr * sp * sy;
    var y = cr * sp * cy + sr * cp * sy;
    var z = cr * cp * sy - sr * sp * cy;

    return { scalar: w, x: x, y: y, z: z };
}

// Convert quaternion to euler angles (degrees, ZYX convention)
function quaternionToEuler(q) {
    var qw = q.scalar;
    var qx = q.x;
    var qy = q.y;
    var qz = q.z;

    // Roll (X-axis rotation)
    var sinr_cosp = 2 * (qw * qx + qy * qz);
    var cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    var roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-axis rotation)
    var sinp = 2 * (qw * qy - qz * qx);
    var pitch;
    if (Math.abs(sinp) >= 1) {
        pitch = (sinp >= 0 ? 1 : -1) * Math.PI / 2;
    } else {
        pitch = Math.asin(sinp);
    }

    // Yaw (Z-axis rotation)
    var siny_cosp = 2 * (qw * qz + qx * qy);
    var cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    var yaw = Math.atan2(siny_cosp, cosy_cosp);

    return {
        x: roll * 180.0 / Math.PI,
        y: pitch * 180.0 / Math.PI,
        z: yaw * 180.0 / Math.PI
    };
}

// Transform a local-frame vector to world frame using quaternion rotation
function transformToWorldFrame(localVec, rotation) {
    var qw = rotation.scalar;
    var qx = rotation.x;
    var qy = rotation.y;
    var qz = rotation.z;

    // Rotation matrix from quaternion
    var r00 = 1 - 2 * (qy * qy + qz * qz);
    var r01 = 2 * (qx * qy - qw * qz);
    var r02 = 2 * (qx * qz + qw * qy);
    var r10 = 2 * (qx * qy + qw * qz);
    var r11 = 1 - 2 * (qx * qx + qz * qz);
    var r12 = 2 * (qy * qz - qw * qx);
    var r20 = 2 * (qx * qz - qw * qy);
    var r21 = 2 * (qy * qz + qw * qx);
    var r22 = 1 - 2 * (qx * qx + qy * qy);

    var vx = localVec.x;
    var vy = localVec.y;
    var vz = localVec.z;

    return {
        x: r00 * vx + r01 * vy + r02 * vz,
        y: r10 * vx + r11 * vy + r12 * vz,
        z: r20 * vx + r21 * vy + r22 * vz
    };
}

// Clamp value between min and max
function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

// Linear interpolation
function lerp(a, b, t) {
    return a + (b - a) * t;
}

// Convert radians to degrees
function radToDeg(rad) {
    return rad * 180.0 / Math.PI;
}

// Convert degrees to radians
function degToRad(deg) {
    return deg * Math.PI / 180.0;
}
