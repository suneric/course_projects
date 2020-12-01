function sample = CreateTrainingSamples(L)
sample = zeros(1,5);

i = 1;
y = -0.01;
x = [-0.8 -0.4 0 0.4];
for j = 1:4
    for angle = (-pi/2):pi/6:0 %  case
        sample(i,:) = [x(j) y x(j)+L*cos(angle) y+L*sin(angle) angle];
        i= i+1;
    end
end

x = [-0.6 -0.2 0.2];
for j = 1:3
    for angle = (-pi/2):pi/6:0 % 8 case
        sample(i,:) = [x(j)+L*cos(angle) y+L*sin(angle) x(j) y angle+pi];
        i= i+1;
    end
end
        
x = [0.6 0.2 -0.2];
for j = 1:3
    for angle = -pi:pi/6:(-pi/2) % 8 case
        sample(i,:) = [x(j) y x(j)+L*cos(angle) y+L*sin(angle) angle];
        i= i+1;
    end
end

x = [0.8 0.4 0 -0.4];
for j = 1:4
    for angle = -pi:pi/6:(-pi/2) % 8 case
        sample(i,:) = [x(j)+L*cos(angle) y+L*sin(angle) x(j) y angle+pi];
        i= i+1;
    end
end

y = -0.3;
angle = -pi/2;
x = [-0.95 0 0.95];
for j = 1:3
    sample(i,:) = [x(j) y x(j)+L*cos(angle) y+L*sin(angle) angle];
    i = i+1;
end

y = -0.98;
angle = pi/2;
x = [-0.5 0.5];
for j = 1:2
    sample(i,:) = [x(j) y x(j)+L*cos(angle) y+L*sin(angle) angle];
    i = i+1;
end

y = -0.5;
x = [-0.99 -0.25 0.49];
for j = 1:3
    sample(i,:) = [x(j) y x(j)+L y 0];
    i = i+1;
end

y = -0.6;
x = [0.99 0.25 -0.49];
for j = 1:3
    sample(i,:) = [x(j) y x(j)-L y -pi];
    i = i+1;
end

end