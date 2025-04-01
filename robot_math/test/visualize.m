fid = fopen("~/ros2_ws/data.txt", "r");
data = fscanf(fid, "%f", [4, inf]);
fclose(fid);
plot(data(1,:), data(2, :), 'r', 'LineWidth', 2);
hold on;
plot(data(1,:), data(3, :), 'b', 'LineWidth', 2);
plot(data(1,:), data(4, :), 'g', 'LineWidth', 2);
hold off;