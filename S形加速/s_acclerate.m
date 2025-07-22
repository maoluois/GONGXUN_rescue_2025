%s形加速，包含初始加速度非零的情况（动态改变目标值）
%由于离散积分和右侧积分的误差累积，导致后半段不能实现平滑收敛。
clc;
a_max = 100;
j_max = 50;
sampletime = 0.01;


t = 0;
a0 = 20;
a = a0;
a_pre = 0;
v_target = 140;
v_current = -120;
v = v_current;
v_delta = v_target - v_current

T = 0;
T0 = 0;
T1 = 0;
T2 = 0;

v_print = [];
t_print = [];
i = 1;
flag = 0;


if v_delta>180
    v_delta = v_delta-360
elseif v_delta <  -180
    v_delta = v_delta+360
end

if a0 == 0
    v_th = a_max^2/j_max
    if abs(v_delta) > v_th
        T1 =a_max/j_max
        T3 = T1
        T2 = (abs(v_delta) - v_th)/a_max
        T = T1+T2+T3
        flag = 10;
    else
        T1 = sqrt(abs(v_delta)/j_max)
        T2 = T1
        flag = 0;
        T = T1+T2
    end
elseif sign(a0) == sign(v_delta)
    v_th = (2*a_max^2 - a0^2)/(2*j_max)
    if abs(v_delta) > v_th
        T1 = (a_max - abs(a0))/j_max
        T3 = a_max/j_max
        T2 = (abs(v_delta) - v_th)/a_max
        T = T1+T2+T3
        flag = 11;
    else
        T1 = (-2*abs(a0) + sqrt(2*a0*a0 + 4*j_max*abs(v_delta)))/(2*j_max)
        T2 = T1 + abs(a0)/j_max
        flag = 1;
        T = T1+T2
    end
else
    T0 = abs(a0)/j_max
    v_new = 0.5*a0*T0;
    v_new = v_delta - v_new
    v_th = a_max^2/j_max
    if abs(v_new) > v_th
        T1 = a_max/j_max
        T3 = T1
        T2 = (abs(v_new) - v_th)/a_max
        T = T1+T2+T3+T0
        flag = 12; 
    else
        T1 = sqrt(abs(v_new)/j_max)
        T2 = T1
        flag = 2;
        T = T1+T2+T0 
    end
end


if flag  == 0 
    while(v_current ~= v_target)
        if(t<=T1)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime
            t = t+sampletime;
        elseif(t>T1 && t<=T2+T1)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime
            t = t+sampletime;
        elseif(t>T2+T1)
        a = 10;
        v_current = v_current + a*sampletime
        t = t+sampletime;
        end
        if( sign(v_current) == sign(v_target))
            if v_delta>0
                if(v_current> v_target)
                  v_current = v_target;
                end
            else
                if(v_current< v_target)
                  v_current = v_target;
                end
            end
        end
        if v_current > 180
            v_current = -180
        elseif v_current < -180
            v_current = 180
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
        if a>a_max
            a = a_max;
        elseif a<-a_max
            a = -a_max;
        end
    end
elseif flag == 1
    while(v_current ~= v_target)  %
        if(t<=T1)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime
            t = t+sampletime;
        elseif(t>T1 && t<=T2+T1)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime
            t = t+sampletime;
        elseif(t>T2+T1)
             if v_delta>0
                a = 30;
            else
                a = -30;
             end
            v_current = v_current + a*sampletime
        end
        if( sign(v_current) == sign(v_target))
            if v_delta>0
                if(v_current> v_target)
                  v_current = v_target;
                end
            else
                if(v_current< v_target)
                  v_current = v_target;
                end
            end
        end
        if v_current > 180
            v_current = -180
        elseif v_current < -180
            v_current = 180
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
        if a>a_max
            a = a_max;
        elseif a<-a_max
            a = -a_max;
        end
    end
elseif flag == 2
    while(v_current~=v_target)
        if(t<=T0)
            a_pre = a;
            a = a - sign(a0)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T0 && t<=T1+T0)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime
            v_current = v_current + 0.5*(a_pre+a)*sampletime
            t = t+sampletime;
        elseif(t>T1+T0 && t<=T2+T1+T0)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T2+T1+T0)
             if v_delta>0
                a = 30;
            else
                a = -30;
             end
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        end
        if( sign(v_current) == sign(v_target))
            if v_delta>0
                if(v_current > v_target)
                  v_current = v_target;
                end
            else
                if(v_current< v_target)
                  v_current = v_target;
                end
            end
        end
        if v_current > 180
            v_current = -180
        elseif v_current < -180
                v_current = 180
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
        if a>a_max
            a = a_max;
        elseif a<-a_max
            a = -a_max;
        end
    end
elseif flag == 10
     while(v_current ~= v_target)
        if(t<=T1)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T1 && t<=T2+T1)
            a = sign(v_delta)*a_max;
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        elseif(t>T1+T2 && t<=T1+T2+T3)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T2+T1+T3)
            a = 10;
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        end
        if( sign(v_current) == sign(v_target))
            if v_delta>0
                if(v_current> v_target)
                  v_current = v_target;
                end
            else
                if(v_current< v_target)
                  v_current = v_target;
                end
            end
        end
        if v_current > 180
            v_current = -180
        elseif v_current < -180
                v_current = 180
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
        if a>a_max
            a = a_max;
        elseif a<-a_max
            a = -a_max;
        end
     end
elseif flag == 11
     while(v_current ~= v_target)  %
        if(t<=T1)
            a_pre = a;
            a = a+sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T1 && t<=T2+T1)
            a = sign(v_delta)*a_max;
            v_current = v_current+a*sampletime;
            t = t+sampletime;
        elseif(t>T1+T2 && t<=T1+T2+T3)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T2+T1+T3)
             if v_delta>0
                a = 30;
            else
                a = -30;
             end
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        end
        if( sign(v_current) == sign(v_target))
            if v_delta>0
                if(v_current> v_target)
                  v_current = v_target;
                end
            else
                if(v_current< v_target)
                  v_current = v_target;
                end
            end
        end
        if v_current > 180
            v_current = -180
        elseif v_current < -180
            v_current = 180
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
        if a>a_max
            a = a_max;
        elseif a<-a_max
            a = -a_max;
        end
    end
elseif flag == 12
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
            a = sign(v_delta)*a_max;
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        elseif(t>T1+T0+T2 && t<=T2+T1+T0+T3)
            a_pre = a;
            a = a-sign(v_delta)*j_max*sampletime;
            v_current = v_current + 0.5*(a_pre+a)*sampletime;
            t = t+sampletime;
        elseif(t>T2+T1+T0+T3)
             if v_delta>0
                a = 30;
            else
                a = -30;
             end
            v_current = v_current + a*sampletime;
            t = t+sampletime;
        end
        if( sign(v_current) == sign(v_target))
            if v_delta>0
                if(v_current > v_target)
                  v_current = v_target;
                end
            else
                if(v_current< v_target)
                  v_current = v_target;
                end
            end
        end
        if v_current > 180
            v_current = -180
        elseif v_current < -180
                v_current = 180
        end
        v_print(i) = v_current;
        t_print(i) = t;
        i = i+1;
        if a>a_max
            a = a_max;
        elseif a<-a_max
            a = -a_max;
        end
    end
end

plot(t_print, v_print);




