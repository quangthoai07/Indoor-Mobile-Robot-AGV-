#ifndef IMU_DATA_H
#define IMU_DATA_H

// Declaration and definition of imu_data_tx
struct imu_data{
	float x, y, z;
};
extern struct imu_data imu_data_tx;
//extern struct imu_data imu_dataY_tx;
//extern struct imu_data imu_dataZ_tx;

#endif // IMU_DATA_H
