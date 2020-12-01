function samples = TraditionalTestSamples()
samples = [
    -0.4  -0.2 -0.4+0.5*cos(-pi) -0.2+0.5*sin(-pi) -pi;
    -0.3 -0.4 -0.3+0.5*cos(-0.8*pi) -0.4+0.5*sin(-0.8*pi) -0.8*pi;
    -0.15 -0.9 -0.15+0.5*cos(0.4*pi) -0.9+0.5*sin(0.4*pi) 0.4*pi;
    0.7 -0.8 0.7+0.5*cos(0.8*pi) -0.8+0.5*sin(0.8*pi) 0.8*pi;
    0.4 -0.3 0.4+0.5*cos(0.05*pi) -0.3+0.5*sin(0.05*pi) 0.05*pi
    ];
end