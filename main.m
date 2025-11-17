clear all;  % clear the previous memory
clc; % clear the previous command

% initialize the robot to mylego
mylego = legoev3('USB');  

%------initialize the motors-------
mymotor_base = motor(mylego,'c');
mymotor_elbow = motor(mylego,'b');
mymotor_wrist = motor(mylego,'a');

%------initialize the sensors------
touchSensor_base = touchSensor(mylego, 1);
touchSensor_elbow = touchSensor(mylego, 3);
distancesensor = sonicSensor(mylego, 2); 

%------Initialize the link parameters
L1 = 50;
L2 = 95;
L3 = 185;
L4 = 110;
L0 = 155;  % height between sensor end and gripper end
global theta2;

%STATION POSITION
S1 = 340;
S2 = -170;
home = 0;

% Task 4: -----Homing Function-------
homing_elbow(mymotor_elbow,touchSensor_elbow);
fake_homing_base(mymotor_base,touchSensor_base);
homing(-340, mymotor_base)
disp(mymotor_base.readRotation());
pause(0.5);
gripperopen_1(mymotor_wrist);
pause(1);

%Task 6: --Pick ball from home and place in Station 1 and return to home--
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperclose_1(mymotor_wrist)
pause(1)
armup(mymotor_elbow, touchSensor_elbow)

turn_motor_base(S1, mymotor_base)
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperopen_1(mymotor_wrist);
pause(1);
armup(mymotor_elbow, touchSensor_elbow)
turn_motor_base(home, mymotor_base)

%Task 7: --move to S1, pick the ball and place it in S2, and return to
%home--
turn_motor_base(S1, mymotor_base)
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperclose_1(mymotor_wrist);
pause(1);
armup(mymotor_elbow, touchSensor_elbow)

turn_motor_base(S2, mymotor_base)
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperopen_1(mymotor_wrist);
pause(1);
armup(mymotor_elbow, touchSensor_elbow)
turn_motor_base(home, mymotor_base)
mymotor_base.resetRotation()

%Task 8: --move to S2, pick the ball and place it in S1, and return to
%home--
turn_motor_base(S2, mymotor_base)
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperclose_1(mymotor_wrist);
pause(1);
armup(mymotor_elbow, touchSensor_elbow)

turn_motor_base(S1, mymotor_base)
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperopen_1(mymotor_wrist);
pause(1);
armup(mymotor_elbow, touchSensor_elbow)
turn_motor_base(home, mymotor_base)

%Task 9: --move to S1, pick the ball and place it in home--
turn_motor_base(S1, mymotor_base)
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperclose_1(mymotor_wrist);
pause(1);
armup(mymotor_elbow, touchSensor_elbow)

turn_motor_base(home, mymotor_base)
theta2 = calculation(distancesensor,L0,L1,L2,L3,L4);
armdown(mymotor_elbow, theta2)
gripperopen_1(mymotor_wrist);
pause(1);
armup(mymotor_elbow, touchSensor_elbow)

% -- Function are written here ---

function turn_motor_base(desired_base_value, motor)
% PID Controller
Kp = 0.18  ;  
Ki = 0.008;    
Kd = 0.08; % Kd = Kd*/dt

sensorValue = motor.readRotation();
error = desired_base_value - sensorValue;
prev_error = error;
pause(1);

integral = 0;
continuePIDFlag = 1;
while continuePIDFlag
    % Error
    sensorValue = motor.readRotation();
    error = desired_base_value - sensorValue;

    % Integral
    integral = integral + error;

    % Derivative
    derivative = (error - prev_error);

    % Control signal
    control_signal = Kp * error + Ki * integral + Kd * derivative;
    motor.Speed = control_signal;
    
    % Error
    prev_error = error;
    sensorValue = motor.readRotation();
    error = desired_base_value - sensorValue;
    if abs(error) <= 5
        pause(0.1);
        prev_error = error;
        sensorValue = motor.readRotation();
        error = desired_base_value - sensorValue;
        if abs(error) <= 5 % another condition - to avoid overshooting
            continuePIDFlag = 0;
        end
    end
end
motor.Speed = 0;
end



function homing(homing_value, motor)
    motor.resetRotation();
    turn_motor_base(homing_value, motor)
    motor.resetRotation();
end


function homing_elbow(motor, sensor)
        % Step 1: start the motor with minimum speed
ispressed_b = readTouch(sensor);
resetRotation(motor);
if ispressed_b ~= 1
    start(motor);
    motor.Speed = -25;
            % Step 2: Run a loop to read the sensor status - if it is 1 then stop the
            % motor & reset the encoder value
    while true
        ispressed_b = readTouch(sensor);
        if ispressed_b == 1
            motor.Speed = 0;
            resetRotation(motor);
        break;
        end
    end
end
end



function fake_homing_base(motor, sensor)
    ispressed = readTouch(sensor);
    resetRotation(motor);
    if ispressed ~= 1
            % Step 1: start the motor with minimum speed
        start(motor);
        motor.Speed = 25;
                % Step 2: Run a loop to read the sensor status - if it is 1 then stop the
                % motor & reset the encoder valu
        while true
            ispressed = readTouch(sensor);
            if ispressed == 1
                motor.Speed = 0;
                resetRotation(motor);
            break;
            end
        end
    end
end


function gripperclose_1(motor)
resetRotation(motor);
start(motor);
motor.Speed = 10;
while true
   rotation = readRotation(motor);
   pause(1);
   if (rotation >= 70)
    motor.Speed = 0;
    break;
    end
end
end


function gripperopen_1(motor)
resetRotation(motor);
start(motor);
motor.Speed = -10;
while true
   rotation = readRotation(motor);
    if (rotation<=-70)  
        motor.Speed = 0;
        break;
    end
end
end


% calculation for Θ:-------
function theta2 = calculation(sensor,h,l1,l2,l3,l4)
disp("Inside theta2 calculation" + newline)
distance = readDistance(sensor);
disp("distance: ");
disp(distance);
z = distance * 1000;
alpha = asind((z-h-l1-l2*sind(45)+l4)/l3);
disp("alpha: ");
disp(alpha);
theta2 = (alpha + 45)*5*0.875;  % Gear ratio of B motor = 5
fprintf('theta2 = %d\n', theta2);
return;
end



function armdown(motor_elbow, theta2)
% PID Controller
Kp = 0.1;  % proportional gain
Ki = 0.005;     % integral gain
Kd = 0;     % derivative gain

sensorValue = motor_elbow.readRotation();
desiredElbowValue = int32(theta2);
error =  desiredElbowValue - sensorValue;
prev_error = error;

integral = 0;
continuePIDFlag = 1;
while continuePIDFlag

    sensorValue = motor_elbow.readRotation();
    error =  desiredElbowValue - sensorValue;

    % Integral
    integral = integral + error;

    % Derivative
    derivative = (error - prev_error);

    % Control signal
    control_signal = Kp * error + Ki * integral + Kd * derivative;
    motor_elbow.Speed = control_signal;
    
    % Error
    sensorValue = motor_elbow.readRotation();
    error = desiredElbowValue - sensorValue;
    prev_error = error;
    if abs(error) <= 5
            continuePIDFlag = 0;
    end
end
motor_elbow.Speed = 0;
end


function armup(motor, sensor)
start(motor);
motor.Speed = -25;
while true
    ispressed_b = readTouch(sensor);
    if ispressed_b == 1
        motor.Speed = 0;
        resetRotation(motor);
    break;
    end
end
end