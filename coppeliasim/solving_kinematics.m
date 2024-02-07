syms a b c d e f g h i j k l v
A = [a b c; d e f; g h i; j k l];
input = [-sqrt(2)/2; sqrt(2)/2; 0];
states = [v;0;-v;0];

solve(states== A*input,b)


% ----------begin user data entry----------

toe_in = 0  % degrees

trackwidth = 0.39522
wheelbase = 0.39522

Xrotate= trackwidth/2;
Yrotate= wheelbase/2;

%-----------end user data entry-----------



htw = trackwidth/2.0;
hwb = wheelbase/2.0;

Xfl=-Xrotate-htw;
Yfl=hwb-Yrotate;

Xfr=htw-Xrotate;
Yfr=hwb-Yrotate;

Xrr=htw-Xrotate;
Yrr=-Yrotate-hwb;

Xrl=-Xrotate-htw;
Yrl=-Yrotate-hwb;


FL=func(Xfl,Yfl,toe_in)
FR=func(Xfr,Yfr,-toe_in)
RR=func(Xrr,Yrr,toe_in)
RL=func(Xrl,Yrl,-toe_in)

max = abs(FL);
if (max<abs(FR))
    max=abs(FR); 
end
if (max<abs(RR)) 
    max=abs(RR); 
end
if (max<abs(RL)) 
    max=abs(RL) 
end


fl=FL/max
fr=FR/max
rr=RR/max
rl=RL/max



function ws = func(Xn,Yn,toe_in)
theta = toe_in/180*pi;
R = sqrt(Xn^2+Yn^2);
alpha = atan2(Yn,Xn)-pi/2+theta;
ws = R*sin(alpha);
end


