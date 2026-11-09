duty = 0.5 * [0 5 10 15 20 30 40 50 60 75]; % duty cycle (%)
speed_left = [0 0 35.7 83.3 125.0 176.6 204.6 228.3 242.3 254.8]; % RPM
speed_right = [0 0 25.1 78.1 121.9 171.4 210.9 221.6 241.5 246.5]; % RPM
figure;
plot(duty, speed_left);
xlabel("Duty Cycle (%)");
ylabel("Speed (RPM)");
hold on;
plot(duty, speed_right);
[left_coeffs, left_stats] = polyfit(duty(2:end), speed_left(2:end), 2);
[right_coeffs, right_stats] = polyfit(duty(2:end), speed_right(2:end), 2);
left_prediction = polyval(left_coeffs, duty);
plot(duty, left_prediction)
right_prediction = polyval(right_coeffs, duty);
plot(duty, right_prediction)
legend("Left Motor", "Right Motor", "Quadratic Prediction of Left Motor", "Quadratic Prediction of Right Motor");
left_coeffs_calc = left_coeffs * 10000;
right_coeffs_calc = right_coeffs * 10000;
left_A = -1 * left_coeffs_calc(1);
left_B = 0.5 * left_coeffs_calc(2);
left_C = (left_B^2 + left_coeffs_calc(1) * left_coeffs_calc(3))/4;
fprintf("Left motor specs: A=%.0f, B=%.0f, C=%.0f\n", left_A, left_B, left_C);
right_A = -1 * right_coeffs_calc(1);
right_B = 0.5 * right_coeffs_calc(2);
right_C = (right_B^2 + right_coeffs_calc(1) * right_coeffs_calc(3))/4;
fprintf("Right motor specs: A=%.0f, B=%.0f, C=%.0f\n\n", right_A, right_B, right_C);

left_linear = polyfit(duty(2:end), speed_left(2:end), 1);
right_linear = polyfit(duty(2:end), speed_right(2:end), 1);
fprintf("Left motor linear gain: %.0f\n", left_linear(1));
fprintf("Right motor linear gain: %.0f\n", right_linear(1));