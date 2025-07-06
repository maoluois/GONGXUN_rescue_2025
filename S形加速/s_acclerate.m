%两段式s形加速，包含初始加速度非零的情况（动态改变目标值）
%由于离散积分和右侧积分的误差累积，导致后半段不能实现平滑收敛。
clc;
a_max = 300;
j_max = 1000;
sampletime = 0.01;


t = 0;
a0 = 0;
a = a0;
a_pre = 60;
v_target = 100;
v_current = 0;
v = v_current;
v_delta = v_target - v_current;

T = 0;
T0 = 0;
T1 = 0;
T2 = 0;

v_print = [];
t_print = [];
i = 1;
flag = 0;


if a0 == 0
    T1 = sqrt(abs(v_delta)/j_max);
    T2 = T1;
    flag = 0;
    T = T1+T2
elseif sign(a0) == sign(v_delta)
    T1 = (-2*abs(a0) + sqrt(2*a0*a0 + 4*j_max*abs(v_delta)))/(2*j_max);
    T2 = T1 + abs(a0)/j_max;
    flag = 1;
    T = T1+T2
else
    T0 = abs(a0)/j_max
    v_new = 0.5*a0*T0;
    v_new = v_delta - v_new;
    T1 = sqrt(abs(v_new)/j_max)
    T2 = T1;
    flag = 2;
    T = T1+T2+T0    
end

if flag  == 0 
    while(v_current ~= v_target)
        if(t<=T1)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T1 && t<=T2+T1)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T2)
        a = 10;
        v_current = v_current + a*sampletime;
        t = t+sampletime;
        end
        if v_delta>0
            if(v_current> v_target)
              v_current = v_target;
            end
        else
            if(v_current< v_target)
              v_current = v_target;
            end
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
    end
elseif(flag == 1)
    while(v_current ~= v_target)  %
        if(t<=T1)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T1 && t<=T2+T1)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T2)
             if v_delta>0
                a = 10;
            else
                a = -10;
             end
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        end
        if v_delta>0
            if(v_current> v_target)
              v_current = v_target;
            end
        else
            if(v_current< v_target)
              v_current = v_target;
            end
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
    end
else
    while(v_current~=v_target)
        if(t<=T0)
            a_pre = a;
            a = a - sign(a0)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T0 && t<=T1+T0)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T1+T0 && t<=T2+T1+T0)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T2+T1+T0)
             if v_delta>0
                a = 20;
            else
                a = -20;
             end
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        end
        if v_delta>0
            if(v_current > v_target)
              v_current = v_target;
            end
        else
            if(v_current< v_target)
              v_current = v_target;
            end
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
    end
end

plot(t_print, v_print);




