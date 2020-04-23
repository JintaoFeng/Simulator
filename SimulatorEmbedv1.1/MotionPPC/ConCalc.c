#include "ConCalc.h"
#include "StateMachine.h"

/*全局函数定义*/

/*
**函数名称：Concalc_Coarse_DX
**输入参数：无
**返回值：  无
**函数描述：粗动台DX反馈控制器和前馈控制器输出。
*/
double Concalc_Coarse_DX()
{
	Stage* pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[CDX].dError = pStage->arrAxis[CDX].dSetPoint - pStage->arrAxis[CDX].dActPos;

	if(pStage->arrAxis[CDX].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[CDX].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[CDX].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDX].sPID_Switch,pStage->arrAxis[CDX].dError);
			}
			else if(pStage->arrAxis[CDX].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDX].sLLC_Switch,pStage->arrAxis[CDX].dError);
			}

			//前馈
			if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].dFeedForwardCoef*pStage->arrAxis[CDX].dAffData;
			}

			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[CDX].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[CDX].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDX].sPID,pStage->arrAxis[CDX].dError);
			}
			else if(pStage->arrAxis[CDX].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDX].sLLC,pStage->arrAxis[CDX].dError);
			}

			//前馈
			if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].dFeedForwardCoef*pStage->arrAxis[CDX].dAffData;
			}
		
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDX].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[CDX].sPID);
		LLCClear(&pStage->arrAxis[CDX].sLLC);
		PIDClear(&pStage->arrAxis[CDX].sPID_Switch);
		LLCClear(&pStage->arrAxis[CDX].sLLC_Switch);
		LLCClear(&pStage->arrAxis[CDX].sLLC_Switch);

		//logMsg("Controler Coarse DX is Open!!\n",0,0,0,0,0,0);
	}
	return(dConCalc);
}
/*
**函数名称：Concalc_Coarse_DY
**输入参数：无
**返回值：  无
**函数描述：粗动台DX反馈控制器和前馈控制器输出。
*/
double Concalc_Coarse_DY()
{
	Stage* pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[CDY].dError = pStage->arrAxis[CDY].dSetPoint - pStage->arrAxis[CDY].dActPos;

	if(pStage->arrAxis[CDY].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[CDY].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[CDY].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDY].sPID_Switch,pStage->arrAxis[CDY].dError);
			}
			else if(pStage->arrAxis[CDY].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDY].sLLC_Switch,pStage->arrAxis[CDY].dError);
			}

			//前馈
			if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].dFeedForwardCoef*pStage->arrAxis[CDY].dAffData;
			}
			
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[CDY].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[CDY].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[CDY].sPID,pStage->arrAxis[CDY].dError);
			}
			else if(pStage->arrAxis[CDY].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[CDY].sLLC,pStage->arrAxis[CDY].dError);
			}
			//前馈
			if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].dFeedForwardCoef*pStage->arrAxis[CDY].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[CDY].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{	
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[CDY].sPID);
		LLCClear(&pStage->arrAxis[CDY].sLLC);
		PIDClear(&pStage->arrAxis[CDY].sPID_Switch);
		LLCClear(&pStage->arrAxis[CDY].sLLC_Switch);
		LLCClear(&pStage->arrAxis[CDY].sLLC_Switch);

		//logMsg("Controler Coarse DY is Open!!\n",0,0,0,0,0,0);
	}
	return(dConCalc);
}
/*
**函数名称：Concalc_Coarse_XTZ
**输入参数：无
**返回值：  无
**函数描述：粗动台XTZ反馈控制器和前馈控制器输出。
*/
double Concalc_Coarse_XTZ()
{
	Stage* pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[XTZ].dError = pStage->arrAxis[XTZ].dSetPoint - pStage->arrAxis[XTZ].dActPos;

	if(pStage->arrAxis[XTZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[XTZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[XTZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[XTZ].sPID_Switch,pStage->arrAxis[XTZ].dError);
			}
			else if(pStage->arrAxis[XTZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[XTZ].sLLC_Switch,pStage->arrAxis[XTZ].dError);
			}
			//前馈
			if(pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].dFeedForwardCoef*pStage->arrAxis[XTZ].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[XTZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[XTZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[XTZ].sPID,pStage->arrAxis[XTZ].dError);
			}
			else if(pStage->arrAxis[XTZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[XTZ].sLLC,pStage->arrAxis[XTZ].dError);
			}
			//前馈
			if(pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[XTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].dFeedForwardCoef*pStage->arrAxis[XTZ].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[XTZ].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[XTZ].sPID);
		LLCClear(&pStage->arrAxis[XTZ].sLLC);
		PIDClear(&pStage->arrAxis[XTZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[XTZ].sLLC_Switch);
		LLCClear(&pStage->arrAxis[XTZ].sFilter);

	}
	return(dConCalc);
}
/*
**函数名称：Concalc_Coarse_YTZ
**输入参数：无
**返回值：  无
**函数描述：粗动台XTZ反馈控制器和前馈控制器输出。
*/
double Concalc_Coarse_YTZ()
{
	Stage * pStage = &Machine.coarseStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[YTZ].dError = pStage->arrAxis[YTZ].dSetPoint - pStage->arrAxis[YTZ].dActPos;

	if(pStage->arrAxis[YTZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[YTZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[YTZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[YTZ].sPID_Switch,pStage->arrAxis[YTZ].dError);
			}
			else if(pStage->arrAxis[YTZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[YTZ].sLLC_Switch,pStage->arrAxis[YTZ].dError);
			}
			//前馈
			if(pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].dFeedForwardCoef*pStage->arrAxis[YTZ].dAffData;
			}

			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_x_Switch*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_y_Switch*pStage->arrAxis[CDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[YTZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[YTZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[YTZ].sPID,pStage->arrAxis[YTZ].dError);
			}
			else if(pStage->arrAxis[YTZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[YTZ].sLLC,pStage->arrAxis[YTZ].dError);
			}
			if(pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[YTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].dFeedForwardCoef*pStage->arrAxis[YTZ].dAffData;
			}
			/*if(pStage->arrAxis[CDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_x*pStage->arrAxis[CDX].dAffData;
			}
			else if(pStage->arrAxis[CDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[CDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[YTZ].SingleFeedForward_y*pStage->arrAxis[CDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[YTZ].sPID);
		LLCClear(&pStage->arrAxis[YTZ].sLLC);
		PIDClear(&pStage->arrAxis[YTZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[YTZ].sLLC_Switch);
		LLCClear(&pStage->arrAxis[YTZ].sFilter);

	}
	return(dConCalc);
}
/*
**函数名称：Concalc_Fine_DX
**输入参数：无
**返回值：  无
**函数描述：微动台DX反馈控制器和前馈控制器输出。
*/
double Concalc_Fine_DX()
{
	Stage* pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FDX].dError = pStage->arrAxis[FDX].dSetPoint - pStage->arrAxis[FDX].dActPos;

	if(pStage->arrAxis[FDX].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FDX].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FDX].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDX].sPID_Switch,pStage->arrAxis[FDX].dError);
			}
			else if(pStage->arrAxis[FDX].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDX].sLLC_Switch,pStage->arrAxis[FDX].dError);
			}

			//前馈
			if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].dFeedForwardCoef*pStage->arrAxis[FDX].dAffData;
			}
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FDX].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FDX].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDX].sPID,pStage->arrAxis[FDX].dError);
			}
			else if(pStage->arrAxis[FDX].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDX].sLLC,pStage->arrAxis[FDX].dError);
			}
				//dConCalc = LLCFilter(&pStage->arrAxis[FDX].sFilter,dConCalc);
				dConCalc = LLCCalculate(&pStage->arrAxis[FDX].sFilter,dConCalc);
			//前馈
			if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].dFeedForwardCoef*pStage->arrAxis[FDX].dAffData;
			}
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_x*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDX].SingleFeedForward_y*pStage->arrAxis[FDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FDX].sPID);
		LLCClear(&pStage->arrAxis[FDX].sLLC);
		PIDClear(&pStage->arrAxis[FDX].sPID_Switch);
		LLCClear(&pStage->arrAxis[FDX].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FDX].sFilter);

	}
	return(dConCalc);
}
/*
**函数名称：Concalc_Fine_DY
**输入参数：无
**返回值：  无
**函数描述：微动台DY反馈控制器和前馈控制器输出。
*/
double Concalc_Fine_DY()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FDY].dError = pStage->arrAxis[FDY].dSetPoint - pStage->arrAxis[FDY].dActPos;

	if(pStage->arrAxis[FDY].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FDY].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FDY].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDY].sPID_Switch,pStage->arrAxis[FDY].dError);
			}
			else if(pStage->arrAxis[FDY].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDY].sLLC_Switch,pStage->arrAxis[FDY].dError);
			}

			//前馈
			if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].dFeedForwardCoef*pStage->arrAxis[FDY].dAffData;
			}
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FDY].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FDY].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDY].sPID,pStage->arrAxis[FDY].dError);
			}
			else if(pStage->arrAxis[FDY].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDY].sLLC,pStage->arrAxis[FDY].dError);
			}
//			dConCalc = LLCFilter(&pStage->arrAxis[FDY].sFilter,dConCalc);
			//前馈
			dConCalc = LLCCalculate(&pStage->arrAxis[FDY].sFilter,dConCalc);
			if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].dFeedForwardCoef*pStage->arrAxis[FDY].dAffData;
			}
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FDY].sPID);
		LLCClear(&pStage->arrAxis[FDY].sLLC);
		PIDClear(&pStage->arrAxis[FDY].sPID_Switch);
		LLCClear(&pStage->arrAxis[FDY].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FDY].sFilter);

	}
	return(dConCalc);
}


/*YHT添加*/
/*
**函数名称：Concalc_Fine_DZ
**输入参数：无
**返回值：  无
**函数描述：微动台DZ反馈控制器和前馈控制器输出。
*/
double Concalc_Fine_DZ()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FDZ].dError = pStage->arrAxis[FDZ].dSetPoint - pStage->arrAxis[FDZ].dActPos;

	if(pStage->arrAxis[FDZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FDZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FDZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDZ].sPID_Switch,pStage->arrAxis[FDZ].dError);
			}
			else if(pStage->arrAxis[FDZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDZ].sLLC_Switch,pStage->arrAxis[FDZ].dError);
			}

			//前馈
			if(pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDZ].dFeedForwardCoef*pStage->arrAxis[FDZ].dAffData;
			}
		}
		else if(pStage->arrAxis[FDZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FDZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FDZ].sPID,pStage->arrAxis[FDZ].dError);
			}
			else if(pStage->arrAxis[FDZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FDZ].sLLC,pStage->arrAxis[FDZ].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FDZ].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FDZ].sFilter,dConCalc);
			//前馈
			if(pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDZ].dFeedForwardCoef*pStage->arrAxis[FDZ].dAffData;
			}
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FDZ].sPID);
		LLCClear(&pStage->arrAxis[FDZ].sLLC);
		PIDClear(&pStage->arrAxis[FDZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[FDZ].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FDZ].sFilter);

	}
	return(dConCalc);
}

/*
**函数名称：Concalc_Fine_TX
**输入参数：无
**返回值：  无
**函数描述：微动台TX反馈控制器和前馈控制器输出。
*/
double Concalc_Fine_TX()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FTX].dError = pStage->arrAxis[FTX].dSetPoint - pStage->arrAxis[FTX].dActPos;

	if(pStage->arrAxis[FTX].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FTX].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FTX].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTX].sPID_Switch,pStage->arrAxis[FTX].dError);
			}
			else if(pStage->arrAxis[FTX].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTX].sLLC_Switch,pStage->arrAxis[FTX].dError);
			}

			//前馈
			if(pStage->arrAxis[FTX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTX].dFeedForwardCoef*pStage->arrAxis[FTX].dAffData;
			}
		}
		else if(pStage->arrAxis[FTX].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FTX].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTX].sPID,pStage->arrAxis[FTX].dError);
			}
			else if(pStage->arrAxis[FTX].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTX].sLLC,pStage->arrAxis[FTX].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FTX].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FTX].sFilter,dConCalc);
			//前馈
			if(pStage->arrAxis[FTX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTX].dFeedForwardCoef*pStage->arrAxis[FTX].dAffData;
			}

		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FTX].sPID);
		LLCClear(&pStage->arrAxis[FTX].sLLC);
		PIDClear(&pStage->arrAxis[FTX].sPID_Switch);
		LLCClear(&pStage->arrAxis[FTX].sLLC_Switch);
		LLCClear(&pStage->arrAxis[FTX].sFilter);

	}
	return(dConCalc);
}

/*
**函数名称：Concalc_Fine_TY
**输入参数：无
**返回值：  无
**函数描述：微动台TY反馈控制器和前馈控制器输出。
*/
double Concalc_Fine_TY()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FTY].dError = pStage->arrAxis[FTY].dSetPoint - pStage->arrAxis[FTY].dActPos;

	if(pStage->arrAxis[FTY].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FTY].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FTY].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTY].sPID_Switch,pStage->arrAxis[FTY].dError);
			}
			else if(pStage->arrAxis[FTY].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTY].sLLC_Switch,pStage->arrAxis[FTY].dError);
			}

			//前馈
			if(pStage->arrAxis[FTY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTY].dFeedForwardCoef*pStage->arrAxis[FTY].dAffData;
			}


			
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FTY].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FTY].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTY].sPID,pStage->arrAxis[FTY].dError);
			}
			else if(pStage->arrAxis[FTY].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTY].sLLC,pStage->arrAxis[FTY].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FTY].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FTY].sFilter,dConCalc);
			//前馈
			if(pStage->arrAxis[FTY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTY].dFeedForwardCoef*pStage->arrAxis[FTY].dAffData;
			}


			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_x*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FDY].SingleFeedForward_y*pStage->arrAxis[FDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FTY].sPID);
		LLCClear(&pStage->arrAxis[FTY].sLLC);
		PIDClear(&pStage->arrAxis[FTY].sPID_Switch);
		LLCClear(&pStage->arrAxis[FTY].sLLC_Switch);
		LLCClear(&(pStage->arrAxis[FTY].sFilter));
	
		}
	return(dConCalc);
}


/*
**函数名称：Concalc_Fine_TZ
**输入参数：无
**返回值：  无
**函数描述：微动台DY反馈控制器和前馈控制器输出。
*/
double Concalc_Fine_TZ()
{
	Stage * pStage = &Machine.fineStage;
	double dConCalc = 0.0;
	
	pStage->arrAxis[FTZ].dError = pStage->arrAxis[FTZ].dSetPoint - pStage->arrAxis[FTZ].dActPos;

	if(pStage->arrAxis[FTZ].axisStatus != S_Aixs_Open)
	{
		if(pStage->arrAxis[FTZ].closeStatus == S_Laser_Close)
		{
			if(pStage->arrAxis[FTZ].LaserControllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTZ].sPID_Switch,pStage->arrAxis[FTZ].dError);
			}
			else if(pStage->arrAxis[FTZ].LaserControllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTZ].sLLC_Switch,pStage->arrAxis[FTZ].dError);
			}


			//前馈
			if(pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].dFeedForwardCoef*pStage->arrAxis[FTZ].dAffData;
			}

			
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_x_Switch*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_y_Switch*pStage->arrAxis[FDY].dAffData;
			}*/
		}
		else if(pStage->arrAxis[FTZ].closeStatus == S_Counter_Close)
		{
			if(pStage->arrAxis[FTZ].controllerType == PID_flag)
			{
				dConCalc = PIDCalculate(&pStage->arrAxis[FTZ].sPID,pStage->arrAxis[FTZ].dError);
			}
			else if(pStage->arrAxis[FTZ].controllerType == LLC_flag)
			{
				dConCalc = LLCCalculate(&pStage->arrAxis[FTZ].sLLC,pStage->arrAxis[FTZ].dError);
			}

//			dConCalc = LLCFilter(&pStage->arrAxis[FTZ].sFilter,dConCalc);
			dConCalc = LLCCalculate(&pStage->arrAxis[FTZ].sFilter,dConCalc);
			//前馈
			if(pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FTZ].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].dFeedForwardCoef*pStage->arrAxis[FTZ].dAffData;
			}

			
			/*if(pStage->arrAxis[FDX].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDX].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_x*pStage->arrAxis[FDX].dAffData;
			}
			else if(pStage->arrAxis[FDY].axisStatus == S_Aixs_RunFor || pStage->arrAxis[FDY].axisStatus == S_Aixs_RunRev)
			{
				dConCalc += pStage->arrAxis[FTZ].SingleFeedForward_y*pStage->arrAxis[FDY].dAffData;
			}*/
		}
	}
	else
	{
		dConCalc = 0.0;
		PIDClear(&pStage->arrAxis[FTZ].sPID);
		LLCClear(&pStage->arrAxis[FTZ].sLLC);
		PIDClear(&pStage->arrAxis[FTZ].sPID_Switch);
		LLCClear(&pStage->arrAxis[FTZ].sLLC_Switch);
		LLCClear(&(pStage->arrAxis[FTZ].sFilter));

	}
	return(dConCalc);
}

double getActVel(double *prePos,double *curPos)
{
	float Ts=0.02;
	//换算单位，mm/s
	double error=0,speed=0;
	error=fabs((*curPos)-(*prePos));
	comData.dCoarseData[61]=*curPos;
	comData.dCoarseData[62]=*prePos;
	comData.dCoarseData[63]=error;
	
	speed = error/Ts;
	*prePos = *curPos;
	comData.dCoarseData[64]=speed;
	return speed;
	
}

void calcSpeed(Axis *pAxis)
{
	pAxis->fVel=(pAxis->dActPos-pAxis->lastActPos)/0.0002;
	pAxis->lastActPos=pAxis->dActPos;
}

void calcAcc(Axis *pAxis)
{
	pAxis->fAcc=(pAxis->fVel - pAxis->lastFVel)/0.0002;
	pAxis->lastFVel=pAxis->fVel;
}


