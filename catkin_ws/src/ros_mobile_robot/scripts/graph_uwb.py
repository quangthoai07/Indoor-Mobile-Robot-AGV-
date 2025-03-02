#!/usr/bin/env python3
import rospy
from localizer_dwm1001.msg import Tag
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from scipy.stats import gaussian_kde

class UWBPlotter:
    def __init__(self):
        rospy.init_node('uwb_plotter', anonymous=True)
        self.uwb_sub = rospy.Subscriber('/dwm1001/tag', Tag, self.uwb_callback)
        
        self.times = []
        self.x_positions = []
        self.y_positions = []
        self.start_time = None

        # Khởi tạo plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 7))
        self.scatter, = self.ax1.plot([], [], 'ro', markersize=3)
        self.line, = self.ax1.plot([], [], 'b-', lw=1)
        
        self.ax1.set_xlabel('Vị trí X (m)')
        self.ax1.set_ylabel('Vị trí Y (m)')
        self.ax1.set_title('Quỹ đạo XY')
        self.ax1.grid()

        self.ax2.set_xlabel('Vị trí X (m)')
        self.ax2.set_ylabel('Vị trí Y (m)')
        self.ax2.set_title('Phân phối nhiễu XY')
        self.ax2.grid()

        self.noise_scatter = self.ax2.scatter([], [], c=[], cmap='viridis')

    def uwb_callback(self, msg):
        if self.start_time is None:
            self.start_time = rospy.Time.now()

        current_time = (rospy.Time.now() - self.start_time).to_sec()
        
        self.times.append(current_time)
        self.x_positions.append(msg.x)
        self.y_positions.append(msg.y)

    def update_plot(self, frame):
        self.scatter.set_data(self.x_positions, self.y_positions)
        self.line.set_data(self.x_positions, self.y_positions)
        
        self.ax1.relim()
        self.ax1.autoscale_view()

        # Cập nhật biểu đồ phân phối nhiễu
        if len(self.x_positions) > 10:  # Đảm bảo có đủ dữ liệu
            xy = np.vstack([self.x_positions, self.y_positions])
            z = gaussian_kde(xy)(xy)

            self.noise_scatter.set_offsets(np.c_[self.x_positions, self.y_positions])
            self.noise_scatter.set_array(z)
            self.ax2.relim()
            self.ax2.autoscale_view()

        return self.scatter, self.line, self.noise_scatter

    def plot_positions(self):
        ani = FuncAnimation(self.fig, self.update_plot, frames=200, interval=50, blit=True)
        plt.tight_layout()
        plt.show()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        uwb_plotter = UWBPlotter()
        uwb_plotter.plot_positions()
        uwb_plotter.run()
    except rospy.ROSInterruptException:
        pass
