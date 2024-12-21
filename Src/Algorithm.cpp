void Kinematics_differential(float wheel1_speed, float wheel2_speed, float wheel_distance, float &linear_speed, float &angular_speed)
{
    linear_speed = (wheel1_speed + wheel2_speed) / 2.0f;
    angular_speed = (wheel2_speed - wheel1_speed) / wheel_distance;
}

void InverseKinematics_differential(float linear_speed, float angular_speed, float &wheel1_speed, float &wheel2_speed, float wheel_distance)
{
    wheel1_speed = linear_speed - angular_speed * wheel_distance / 2.0f;
    wheel2_speed = linear_speed + angular_speed * wheel_distance / 2.0f;
}