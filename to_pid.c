// 标准形式 PID 算法

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    return kp * error + ki * integral + kd * derivative;
}

// 增量式 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral    = 0;
    static f32 prev_input  = 0;
    f32        error       = setpoint - input;
    f32        delta_input = input - prev_input;
    integral += error * dt;
    f32 derivative = delta_input / dt;
    prev_input     = input;
    return kp * error + ki * integral + kd * derivative;
}

// 平衡式 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += (error + prev_error) * dt / 2;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    return kp * error + ki * integral + kd * derivative;
}

// 平方根形式
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    return kp * sqrt(fabs(error)) * (error > 0 ? 1 : -1) + ki * integral + kd * derivative;
}

// 位置型 P

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    return kp * error + ki * integral + kd * derivative;
}

// 增量型

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral    = 0;
    static f32 prev_input  = 0;
    static f32 prev_output = 0;
    f32        error       = setpoint - input;
    f32        delta_input = input - prev_input;
    integral += error * dt;
    f32 delta_output = kp * (error - prev_error) + ki * integral - kd * delta_input;
    f32 output       = prev_output + delta_output;
    prev_input       = input;
    prev_output      = output;
    prev_error       = error;
    return output;
}

// 线性逼近 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 离散积分 PID 算法：

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += (error + prev_error) * dt / 2;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    return kp * error + ki * integral + kd * derivative;
}

// 反馈限幅 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
}

// 位置型反馈限幅 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt, f32 max_output)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    if (output > max_output) output = max_output;
    if (output < -max_output) output = -max_output;
    return output;
}

// 参考模型自适应 PID 算法：

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral        = 0;
    static f32 prev_error      = 0;
    static f32 prev_input      = 0;
    static f32 prev_derivative = 0;
    static f32 model_output    = 0;
    static f32 model_error     = 0;
    f32        error           = setpoint - input;
    integral += error * dt;
    f32 derivative       = (input - prev_input) / dt;
    f32 model_derivative = (derivative - prev_derivative) / dt;
    prev_error           = error;
    prev_input           = input;
    prev_derivative      = derivative;
    f32 output           = kp * error + ki * integral + kd * derivative + model_output;
    model_output         = model_error * kp + model_derivative * kd;
    model_error          = error;
    return output;
}

// 离散微分 PID 算法

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral        = 0;
    static f32 prev_error      = 0;
    static f32 prev_derivative = 0;
    f32        error           = setpoint - input;
    integral += error * dt;
    f32 derivative  = (error - prev_error) / dt;
    prev_error      = error;
    f32 output      = kp * error + ki * integral + kd * (2 * kd / dt * derivative - prev_derivative);
    prev_derivative = 2 * kd / dt * derivative - prev_derivative;
    return output;
}

// 平滑积分 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral              = integral * 0.9 + error * dt;
    f32 derivative        = (error - prev_error) / dt;
    prev_error            = error;
    return kp * error + ki * integral + kd * derivative;
}

// 带限制积分 PID 算法：

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    if (integral < -1.0 / ki) integral = -1.0 / ki;
    if (integral > 1.0 / ki) integral = 1.0 / ki;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    return kp * error + ki * integral + kd * derivative;
}

// 非线性 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * tanh(ki * integral) + kd * derivative;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 非线性比例积分 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * pow(fabs(error), 0.5) * (error > 0 ? 1 : -1) + ki * pow(fabs(integral), 0.5) * (integral > 0 ? 1 : -1) + kd * derivative;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 双环 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp1, f32 ki1, f32 kd1, f32 kp2, f32 ki2, f32 kd2, f32 dt)
{
    static f32 integral1   = 0;
    static f32 prev_error1 = 0;
    static f32 integral2   = 0;
    static f32 prev_error2 = 0;
    f32        error1      = setpoint - input;
    integral1 += error1 * dt;
    f32 derivative1 = (error1 - prev_error1) / dt;
    prev_error1     = error1;
    f32 output1     = kp1 * error1 + ki1 * integral1 + kd1 * derivative1;
    f32 error2      = output1 - input;
    integral2 += error2 * dt;
    f32 derivative2 = (error2 - prev_error2) / dt;
    prev_error2     = error2;
    f32 output2     = kp2 * error2 + ki2 * integral2 + kd2 * derivative2;
    if (output2 > 1.0) output2 = 1.0;
    if (output2 < -1.0) output2 = -1.0;
    return output2;
}

// 多项式 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral        = 0;
    static f32 prev_error      = 0;
    static f32 prev_derivative = 0;
    f32        error           = setpoint - input;
    integral += error * dt;
    f32 derivative  = (error - prev_error) / dt;
    f32 output      = kp * error + ki * integral + kd * derivative + kp * pow(error, 2) + ki * pow(integral, 2) + kd * pow(derivative, 2) + kp * pow(error, 3) + ki * pow(integral, 3) + kd * pow(derivative, 3);
    prev_error      = error;
    prev_derivative = derivative;
    return output;
}

// 模糊 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 error_history[3] = {0, 0, 0};
    static f32 integral         = 0;
    static f32 prev_error       = 0;
    f32        error            = setpoint - input;
    integral += error * dt;
    f32 derivative   = (error - prev_error) / dt;
    prev_error       = error;
    error_history[2] = error_history[1];
    error_history[1] = error_history[0];
    error_history[0] = error;
    f32 output       = kp * error + ki * integral + kd * derivative + kp * (error - error_history[1]) + ki * error_history[0] + kd * (error - 2 * error_history[1] + error_history[2]);
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 自适应模型预测控制 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral     = 0;
    static f32 prev_error   = 0;
    static f32 model_output = 0;
    static f32 model_error  = 0;
    f32        error        = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative + model_output;
    model_output   = model_error * kp + ki * error + (model_error - error) * kd / dt;
    model_error    = error;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 基于神经网络的 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 weights[3]      = {0.5, 0.3, 0.2};
    static f32 prev_error      = 0;
    static f32 prev_integral   = 0;
    static f32 prev_derivative = 0;
    f32        error           = setpoint - input;
    f32        integral        = prev_integral + error * dt;
    f32        derivative      = (error - prev_error) / dt;
    prev_error                 = error;
    prev_integral              = integral;
    prev_derivative            = derivative;
    f32 output                 = kp * error + ki * integral + kd * derivative;
    f32 input_layer[3]         = {error, integral, derivative};
    f32 hidden_layer[3]        = {0};
    f32 output_layer           = 0;
    for (int i = 0; i < 3; i++) {
        hidden_layer[i] = weights[i] * input_layer[i];
        output_layer += hidden_layer[i];
    }
    if (output_layer > 1.0) output_layer = 1.0;
    if (output_layer < -1.0) output_layer = -1.0;
    return output + output_layer;
}

// 引入死区的 PID 算法

f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    if (fabs(error) < 0.1) error = 0;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 反向控制的 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = input - setpoint;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 滑动模式控制的 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output + kd * (error - prev_error) / dt;
}

// 二阶滞后 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral    = 0;
    static f32 prev_error  = 0;
    static f32 prev_output = 0;
    f32        error       = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    output         = 2 * prev_output / (1 + exp(-output)) - prev_output;
    prev_output    = output;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 非线性反馈的 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    output += sin(output);
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 带有饱和控制的 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    f32 saturation = 0.5;
    if (output > saturation) {
        output = saturation;
        integral -= error * dt;
    }
    if (output < -saturation) {
        output = -saturation;
        integral -= error * dt;
    }
    return output;
}

// 预测控制PID算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt, f32 tau)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative       = (error - prev_error) / dt;
    prev_error           = error;
    f32 output           = kp * error + ki * integral + kd * derivative;
    f32 prediction       = input + tau * derivative;
    f32 prediction_error = setpoint - prediction;
    output += kp * prediction_error;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 带有动态阈值的PID算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    static f32 threshold  = 1.0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    f32 abs_output = fabs(output);
    if (abs_output > threshold) {
        output = threshold * output / abs_output;
    }
    threshold = 1.0 / (1.0 + exp(-ki * integral));
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 带有模型预测的PID算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt, f32 tau)
{
    static f32 integral    = 0;
    static f32 prev_error  = 0;
    static f32 prev_output = 0;
    f32        error       = setpoint - input;
    integral += error * dt;
    f32 derivative       = (error - prev_error) / dt;
    prev_error           = error;
    f32 output           = kp * error + ki * integral + kd * derivative;
    f32 prediction       = input + tau * prev_output;
    f32 prediction_error = setpoint - prediction;
    output += kp * prediction_error;
    prev_output = output;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 带有鲁棒性控制的 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt, f32 alpha)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    f32 feedback   = alpha * input;
    output += feedback;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}

// 带有自适应控制的 PID 算法
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral    = 0;
    static f32 prev_error  = 0;
    static f32 prev_output = 0;
    static f32 alpha       = 1.0;
    f32        error       = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    output += alpha * (output - prev_output);
    prev_output  = output;
    f32 feedback = alpha * input;
    output += feedback;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    alpha += ki * fabs(error) * dt;
    if (alpha > 1.0) alpha = 1.0;
    return output;
}

// 带有模糊控制的 PID 算法：
f32 pid(f32 setpoint, f32 input, f32 kp, f32 ki, f32 kd, f32 dt)
{
    static f32 integral   = 0;
    static f32 prev_error = 0;
    f32        error      = setpoint - input;
    integral += error * dt;
    f32 derivative = (error - prev_error) / dt;
    prev_error     = error;
    f32 output     = kp * error + ki * integral + kd * derivative;
    f32 feedback   = 0.0;
    if (error < -1.0) {
        feedback = 1.0;
    } else if (error > 1.0) {
        feedback = -1.0;
    }
    output += feedback;
    if (output > 1.0) output = 1.0;
    if (output < -1.0) output = -1.0;
    return output;
}