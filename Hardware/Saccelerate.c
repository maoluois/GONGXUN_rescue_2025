#include "Saccelerate.h"
#define sampletime 0.01

void Sacc_Init(sacc_typedef* sacc, float j_max, float a_max, float a_supplement, float minimun)
{
    sacc->j_max = j_max;
    sacc->a_max = a_max;
    sacc->a_supplement = a_supplement;
    sacc->mininum = minimun;
}

void Angle_Constrain(sacc_typedef* sacc)
{
    sacc->delta = sacc->target - sacc->current;
    if(sacc->delta > 180)
        sacc->delta -= 360;
    else if(sacc->delta < -180)
        sacc->delta += 360;
}


void time_calculate(sacc_typedef* sacc)
{
    if(fabs(sacc->delta) < sacc->mininum)
    {
        sacc->time_flag = -1;
        return;
    }
        
    sacc->a0 = sacc->a;
    if(sacc->a0 == 0)
    {
        sacc->threshold = sacc->a_max*sacc->a_max/sacc->j_max;
        if(fabs(sacc->delta)>sacc->threshold)
        {
            sacc->t1 = sacc->a_max/sacc->j_max;
            sacc->t3 = sacc->t1;
            sacc->t2 = (fabs(sacc->delta) - sacc->threshold)/sacc->a_max;
            sacc->t = sacc->t1+sacc->t2+sacc->t3;
            sacc->time_flag = 11;
        }
        else
        {
            sacc->t1 = fast_sqrt(fabs(sacc->delta)/sacc->j_max);
            sacc->t2 = sacc->t1;
            sacc->t = sacc->t1 + sacc->t2;
            sacc->time_flag = 1;
        }
    }
    else if(signbit(sacc->a0) == signbit(sacc->delta))
    {
        sacc->threshold = (2*sacc->a_max*sacc->a_max - sacc->a0*sacc->a0)/(2*sacc->j_max);
        if(fabs(sacc->delta) > sacc->threshold)
        {
            sacc->t1 = (sacc->a_max - fabs(sacc->a0))/sacc->j_max;
            sacc->t3 = sacc->a_max /sacc->j_max;
            sacc->t2 = (fabs(sacc->delta) - sacc->threshold)/sacc->a_max;
            sacc->t = sacc->t1+sacc->t2+sacc->t3;
            sacc->time_flag = 12;
        }
        else
        {
            sacc->t1 = (-2*fabs(sacc->a0) + fast_sqrt(2*sacc->a0*sacc->a0 + 4*sacc->j_max*fabs(sacc->delta)))/(2*sacc->j_max);
            sacc->t2 = sacc->t1+fabs(sacc->a0)/sacc->j_max;
            sacc->t = sacc->t1 + sacc->t2;
            sacc->time_flag = 2;
        }
    }
    else
    {
        sacc->t0 = fabs(sacc->a0)/sacc->j_max;
        sacc->vnew = 0.5*sacc->a0*sacc->t0;
        sacc->vnew = sacc->delta - sacc->vnew;
        sacc->threshold = sacc->a_max*sacc->a_max/sacc->j_max;
        if(fabs(sacc->vnew)>sacc->threshold)
        {
            sacc->t1 = sacc->a_max/sacc->j_max;
            sacc->t3 = sacc->t1;
            sacc->t2 = (fabs(sacc->vnew) - sacc->threshold)/sacc->a_max;
            sacc->t = sacc->t1+sacc->t2+sacc->t3+sacc->t0;
            sacc->time_flag = 13;
        }
        else
        {
            sacc->t1 = fast_sqrt(fabs(sacc->vnew)/sacc->j_max);
            sacc->t2 = sacc->t1;
            sacc->t = sacc->t1 + sacc->t2+sacc->t0;
            sacc->time_flag = 3;
        }
    }
    sacc->tick = 0.0;
}



void acc_calculate(sacc_typedef* sacc)
{
    if(sacc->time_flag == -1)
        sacc->current = sacc->target;
    if(sacc->current != sacc->target)
    {
        switch(sacc->time_flag)
        {
            case 1:
                if(sacc->tick <= sacc->t1)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a += sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1 && sacc->tick<=sacc->t1+sacc->t2)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else
                {
                    sacc->delta>0 ? (sacc->a = sacc->a_supplement) : (sacc->a = -sacc->a_supplement);
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                break;
            case 2:
                if(sacc->tick <= sacc->t1)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a += sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1 && sacc->tick<=sacc->t1+sacc->t2)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else
                {
                    sacc->delta>0 ? (sacc->a = sacc->a_supplement) : (sacc->a = -sacc->a_supplement);
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                break;
            case 3:
                if (sacc->tick <= sacc->t0)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->a0)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t0 && sacc->tick <= sacc->t1+sacc->t0)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a += sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1+sacc->t0 && sacc->tick<=sacc->t1+sacc->t2+sacc->t0)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else
                {
                    sacc->delta>0 ? (sacc->a = sacc->a_supplement) : (sacc->a = -sacc->a_supplement);
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                break;
            case 11:
                if(sacc->tick <= sacc->t1)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a += sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1 && sacc->tick<=sacc->t1+sacc->t2)
                {
                    sacc->a = sign(sacc->delta)*sacc->a_max;
                    sacc->current += sacc->a*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1+sacc->t2 && sacc->tick<sacc->t1+sacc->t2+sacc->t3)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else
                {
                    sacc->delta>0 ? (sacc->a = sacc->a_supplement) : (sacc->a = -sacc->a_supplement);
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                break;
            case 12:
                if(sacc->tick <= sacc->t1)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a += sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1 && sacc->tick<=sacc->t1+sacc->t2)
                {
                    sacc->a = sign(sacc->delta)*sacc->a_max;
                    sacc->current += sacc->a*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1+sacc->t2 && sacc->tick<sacc->t1+sacc->t2+sacc->t3)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else
                {
                    sacc->delta>0 ? (sacc->a = sacc->a_supplement) : (sacc->a = -sacc->a_supplement);
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                break;
            case 13:
                if (sacc->tick <= sacc->t0)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->a0)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t0 && sacc->tick <= sacc->t1+sacc->t0)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a += sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                } 
                else if(sacc->tick>sacc->t1+sacc->t0 && sacc->tick<=sacc->t1+sacc->t2+sacc->t0)
                {
                    sacc->a = sign(sacc->delta)*sacc->a_max;
                    sacc->current += sacc->a*sampletime;
                    sacc->tick += sampletime;
                }
                else if(sacc->tick>sacc->t1+sacc->t2+sacc->t0 && sacc->tick<=sacc->t1+sacc->t2+sacc->t3+sacc->t0)
                {
                    sacc->a_pre = sacc->a;
                    sacc->a -= sign(sacc->delta)*sacc->j_max*sampletime;
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                else
                {
                    sacc->delta>0 ? (sacc->a = sacc->a_supplement) : (sacc->a = -sacc->a_supplement);
                    sacc->current += 0.5*(sacc->a_pre + sacc->a)*sampletime;
                    sacc->tick += sampletime;
                }
                break;
        }
        if(sign(sacc->current) == sign(sacc->target))
        {
            if (sacc->delta>0)
            {
                if(sacc->current>sacc->target)
                    sacc->current = sacc->target;
            }
            else
            {
                if(sacc->current<sacc->target)
                    sacc->current = sacc->target;
            }
        }
        if(sacc->a > sacc->a_max)
            sacc->a = sacc->a_max;
        else if(sacc->a < -sacc->a_max)
            sacc->a = -sacc->a_max;
    }
    else
        sacc->a = 0;
    
}


float fast_sqrt(float a)
{
    float x;
    if (a == 0)
        return 0.0;
    else
    {
        x = (a<1.0)?(a+1.0)*0.5:a*0.5;
        x = 0.5*(x+a/x);
        x = 0.5*(x+a/x);
        x = 0.5*(x+a/x);
        x = 0.5*(x+a/x);
        return x;
    }
}

int sign(float a)
{
    if(a<0)
        return -1;
    else
        return 1;
}
