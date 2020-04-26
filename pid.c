#include <stdio.h>
#include <strings.h>
#include <stdlib.h>

//pid strcuture
struct pid_params_t
{
	float expect;
	float actual;

	float kp;
	float ki;
	float kd;
	float kf;
	float ff_compensate;//feedforward

	float err;
	float last_err;
	float intergrate;

	float output;

	float intergrate_max;
	float output_max;
	float output_min;
};

//initialize pid
void init_pid(struct pid_params_t *pid)
{
	pid->expect 	= 200.0;
	pid->actual 	= 0;
	pid->kp 		= 0.6;
	pid->ki 		= 0.3;
	pid->kd 		= 0.0;
	pid->err 		= 0;
	pid->last_err 	= 0;
	pid->intergrate = 0;
	pid->output 	= 0;
	pid->intergrate_max = 200;
	pid->output_max     = 200;
	pid->output_min     = -200;
	pid->kf				= 1;
	pid->ff_compensate	= 0;
}


//position pid
void realize_pid(struct pid_params_t *pid)
{
	pid->err         = pid->expect - pid->actual;
	pid->intergrate += pid->err;
	pid->output      = pid->kp * pid->err + \
				  	   pid->ki * pid->intergrate +  \
				       pid->kd * (pid->err - pid->last_err);
	pid->last_err    = pid->err;
	printf("actual: %f.  err: %f. output: %f\n",pid->actual,pid->err,pid->output);
	pid->actual      = pid->output * 1;
}

//intergral separation pid
void realize_pid_intergral_separation(struct pid_params_t *pid)
{
	pid->err         = pid->expect - pid->actual;

	if(abs(pid->err) > pid->intergrate_max){
		pid->intergrate = 0;
	}else{
		pid->intergrate += pid->err;
	}

	pid->output      = pid->kp * pid->err + \
				  	   pid->ki * pid->intergrate +  \
				       pid->kd * (pid->err - pid->last_err);
	pid->last_err    = pid->err;
	printf("actual: %f.  err: %f. output: %f\n",pid->actual,pid->err,pid->output);
	pid->actual      = pid->output * 1;
}

//prevent intergral saturation pid
void realize_pid_intergral_saturation(struct pid_params_t *pid)
{
	pid->err         = pid->expect - pid->actual;

	if(pid->output > pid->output_max){
		if(abs(pid->err) > pid->intergrate_max){
			pid->intergrate = 0;
		}else{
			if(pid->err < 0) pid->intergrate += pid->err;
		}
	}else if(pid->output < pid->output_min){
		if(abs(pid->err) > pid->intergrate_max){
			pid->intergrate = 0;
		}else{
			if(pid->err > 0) pid->intergrate += pid->err;
		}
	}else{
		if(abs(pid->err) > pid->intergrate_max){
			pid->intergrate = 0;
		}else{
			pid->intergrate += pid->err;
		}
	}

	pid->output      = pid->kp * pid->err + \
				  	   pid->ki * pid->intergrate +  \
				       pid->kd * (pid->err - pid->last_err);
	pid->last_err    = pid->err;
	printf("actual: %f.  err: %f. output: %f\n",pid->actual,pid->err,pid->output);
	pid->actual      = pid->output * 1;
}

//variable intergral pid
void realize_pid_intergral_variable(struct pid_params_t *pid)
{
	pid->err         = pid->expect - pid->actual;

	float index = 0;
	if(abs(pid->err) > 200){ //variable intergral
		index = 0;
	}else if(abs(pid->err) < 50){
		index = 1;
	}else{
		index = (abs(pid->err) - 50) / (200 - 50);
	}

	pid->output      = pid->kp * pid->err + \
				  	   pid->ki * pid->intergrate * index +  \
				       pid->kd * (pid->err - pid->last_err);
	pid->last_err    = pid->err;
	printf("actual: %f.  err: %f. output: %f\n",pid->actual,pid->err,pid->output);
	pid->actual      = pid->output * 1;
}

//fuzzy pid
float E[7]  = {-0.6,-0.4,-0.2,0,0.2,0.4,0.6};
float EC[7] = {-0.3,-0.2,-0.1,0,0.1,0.2,0.3};

//kp:[0.5,1]
float rule_kp[7][7] = 
{
	{120,110,100,90,80,70,60},
	{120,110,100,90,80,70,60},
	{120,110,100,90,80,70,60},
	{120,110,100,90,80,70,60},
	{120,110,100,90,80,70,60},
	{120,110,100,90,80,70,60},
	{120,110,100,90,80,70,60}
};

void realize_pid_fuzzy(struct pid_params_t *pid)
{


}

//position pid + feedforward
void realize_pid_ff(struct pid_params_t *pid)
{
	pid->err         = pid->expect - pid->actual;
	pid->intergrate += pid->err;
	pid->output      = pid->kp * pid->err + \
				  	   pid->ki * pid->intergrate +  \
				       pid->kd * (pid->err - pid->last_err);

	pid->output      += pid->kf * pid->ff_compensate;//
	pid->last_err    = pid->err;
	printf("actual: %f.  err: %f. output: %f\n",pid->actual,pid->err,pid->output);
	pid->actual      = pid->output * 1;
}


//global data
struct pid_params_t g_pid;

int main()
{
	init_pid(&g_pid);

	int cnt = 0;
	while (cnt < 100)
	{
		//realize_pid(&g_pid);
		//realize_pid_intergral_separation(&g_pid);
		//realize_pid_intergral_saturation(&g_pid);
		realize_pid_intergral_variable(&g_pid);
		cnt++;
	}
	return 0;
}
