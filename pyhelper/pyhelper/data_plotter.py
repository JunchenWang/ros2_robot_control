import rclpy
from rclpy.node import Node
from robot_msgs.msg import VectorData
import matplotlib.pyplot as plt
from collections import deque
import math

class DynamicDataPlotter(Node):
    def __init__(self, topic_names):
        super().__init__('dynamic_data_plotter')

        self.topic_names = topic_names
        # 每个话题对应一个字典，key 为 data 索引，value 为对应的数据队列
        self.data_deques = {topic: {} for topic in topic_names}

        # 动态订阅所有话题
        self.subscribers = []
        for topic in topic_names:
            subscriber = self.create_subscription(VectorData, topic, self.create_callback(topic), 10)
            self.subscribers.append(subscriber)

        # 自动计算子图的行数和列数
        n_topics = len(topic_names)
        ncols = 2  # 每行最多显示2个图
        nrows = math.ceil(n_topics / ncols)  # 根据话题数量计算行数

        # Matplotlib 图表初始化
        self.fig, self.axs = plt.subplots(nrows, ncols, figsize=(10, 5 * nrows))

        # 检查 axs 的类型并做相应处理
        if n_topics == 1:  # 如果只有一个话题，axs不是列表，需要转为列表
            self.axs = [self.axs]
        elif n_topics == 2:  # 如果有两个图，变为一行两列
            self.axs = self.axs.flatten()  # 手动扁平化 axs
        else:  # 对于更多的图，使用 flatten
            self.axs = self.axs.flatten()

        self.lines = {}  # 存储每个话题的曲线
        self.titles = {}  # 存储每个话题的标题，用于动态更新
        for idx, (ax, topic) in enumerate(zip(self.axs, topic_names)):  # 此处直接使用 self.axs
            self.lines[topic] = {}
            ax.set_xlabel('Timestamp', fontsize=16)  # 设置X轴标签字体大小
            ax.set_ylabel('Data', fontsize=16)  # 设置Y轴标签字体大小
            ax.legend(fontsize=15)  # 设置图例字体大小
            ax.tick_params(axis='both', labelsize=15)  # 设置坐标轴刻度标签字体大小
            ax.grid(True)  # 启用网格
            self.titles[topic] = ax.text(
                0.5, 1.1, f"", fontsize=20, ha='center', transform=ax.transAxes
            )  # 初始化标题

        # 调整子图间距，避免标题和坐标轴重叠
        self.fig.subplots_adjust(hspace=0.4, wspace=0.3)

        # 启动定时更新
        self.timer = self.create_timer(0.001, self.update_plot)

        # 在后台启动输入监听线程
        self.listen_for_input()

    def listen_for_input(self):
        """启动一个监听线程来接收用户输入的命令"""
        import threading
        threading.Thread(target=self.input_listener, daemon=True).start()

    def input_listener(self):
        while rclpy.ok():
            command = input("Enter '1' to adjust axis range: ")
            if command == "1":
                self.adjust_axis_range()

    def adjust_axis_range(self):
        """调整当前坐标轴范围为已有数据的范围，并增加缓冲区"""
        for ax in self.axs.flatten():  # 遍历所有轴
            # 获取数据的最大最小值
            all_timestamps = []
            all_values = []
            for topic in self.lines:
                for i in self.lines[topic]:
                    timestamps, values = zip(*self.data_deques[topic][i]) if self.data_deques[topic][i] else ([], [])
                    all_timestamps.extend(timestamps)
                    all_values.extend(values)

            # 设置新的坐标轴范围，并增加缓冲区
            if all_timestamps and all_values:
                timestamp_min, timestamp_max = min(all_timestamps), max(all_timestamps)
                value_min, value_max = min(all_values), max(all_values)

                # 计算缓冲区
                timestamp_range = timestamp_max - timestamp_min
                value_range = value_max - value_min

                buffer_factor = 0.1  # 缓冲区比例，可以调整
                ax.set_xlim(timestamp_min - buffer_factor * timestamp_range, timestamp_max + buffer_factor * timestamp_range)
                ax.set_ylim(value_min - buffer_factor * value_range, value_max + buffer_factor * value_range)

        plt.draw()  # 强制刷新图形

    def create_callback(self, topic):
        def callback(msg):
            # 自动检测 data 长度，并为每个数据分量初始化队列
            data_len = len(msg.data)
            if not self.data_deques[topic]:  # 第一次接收到消息时初始化
                for i in range(data_len):
                    self.data_deques[topic][i] = deque(maxlen=5000)

                # 初始化 Matplotlib 曲线
                ax = self.axs.flatten()[self.topic_names.index(topic)]  # 扁平化ax数组
                for i in range(data_len):
                    line, = ax.plot([], [], label=f'{msg.name}[{i}]')  # 为每个分量添加曲线
                    self.lines[topic][i] = line
                ax.legend(fontsize=12)  # 设置图例字体大小

            # 将数据存入对应队列
            for i in range(data_len):
                self.data_deques[topic][i].append((msg.timestamp, msg.data[i]))

            # 动态更新标题为消息的 name
            self.titles[topic].set_text(f"{msg.name}")
        return callback

    def update_plot(self):
        for topic, topic_lines in self.lines.items():
            for i, line in topic_lines.items():
                timestamps, values = zip(*self.data_deques[topic][i]) if self.data_deques[topic][i] else ([], [])
                line.set_data(timestamps, values)  # 更新横纵坐标数据

        for ax in self.axs.flatten():  # 遍历所有轴
            ax.relim()  # 更新轴的限制
            ax.autoscale_view()  # 自动调整视图

        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)

    # 在这里修改话题名列表
    topic_names = ["plot1", "plot2", "plot3", "plot4"]  # 示例
    plotter = DynamicDataPlotter(topic_names)

    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
