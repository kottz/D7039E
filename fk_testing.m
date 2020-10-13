d1 = 75;
alpha1 = -pi/2;
a2 = 67.5;
a3 = 67.5;
a4 = 65;

L1 = Link('d', d1, 'a', 0, 'alpha', -pi/2);
L2 = Link('d', 0, 'a', a2, 'alpha', 0);
L3 = Link('d', 0, 'a', a3, 'alpha', 0);

bot2 = SerialLink([L1 L2], 'name', 'Robot Arm'); 
bot3 = SerialLink([L1 L2 L3], 'name', 'Robot Arm');


theta = [1, -0.5, 0];
alpha = [-pi/2, 0, 0];
d = [d1, 0, 0];
a = [0, a2, a3];

A = zeros(4,4,3);
for index = 1:length(theta)
    A(:,:,index) = [cos(theta(index)), -sin(theta(index)) * cos(alpha(index)), sin(theta(index)) * sin(alpha(index)), a(index) * cos(theta(index)) ; ...
          sin(theta(index)), cos(theta(index)) * cos(alpha(index)), -cos(theta(index)) * sin(alpha(index)), a(index) * sin(theta(index)) ; ...
          0, sin(alpha(index)), cos(alpha(index)), d(index) ; ...
          0, 0, 0, 1];
end

A_tot = A(:,:,1)*A(:,:,2)*A(:,:,3)


bot3.plot(theta)



