#include "math.h"
struct MPU6050RPY {
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
};

struct Quaternion {
    float w, x, y, z;
};

struct Vector3D {
    float  x, y, z;
};

struct ImuMeansurements{
    Quaternion orientation;
    Vector3D gyration;
    Vector3D acceleration;
};

Quaternion ToQuaternion(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5);
    float sr = sinf(roll * 0.5);
    float cp = cosf(pitch * 0.5);
    float sp = sinf(pitch * 0.5);
    float cy = cosf(yaw * 0.5);
    float sy = sinf(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

void normalize(Quaternion* quaternion) {
    float magnitude = sqrt(quaternion->x * quaternion->x + quaternion->y * quaternion->y +
                            quaternion->z * quaternion->z + quaternion->w * quaternion->w);

    quaternion->x /= magnitude;
    quaternion->y /= magnitude;
    quaternion->z /= magnitude;
    quaternion->w /= magnitude;
}