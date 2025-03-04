
#ifndef _FUNCTION_H
#define _FUNCTION_H

/*-----------------------------------------------------------------------------------*/

#include <stdio.h>
#include <windows.h>

/*-----------------------------------------------------------------------------------*/
/* Type Definitions of Variables											         */
/*-----------------------------------------------------------------------------------*/
/* COUNT : temporary count													         */
/* LOGIC : returens of logic check results of function (NOK, YOK, and so on)         */
/* FLAG  : flag (NOK, YOK)													         */
/* REAL  : float or doulbe constant or array								         */
/*-----------------------------------------------------------------------------------*/

typedef long int				COUNT           ;
typedef unsigned int			INDEX, NSIZE    ;
typedef	unsigned short			LOGIC , FLAG  , LIDINT  ;
typedef	double					REAL            ;
typedef long				    LIDATA          ;

/*-----------------------------------------------------------------------------------*/
/* Array Length																         */
/*-----------------------------------------------------------------------------------*/

#define BUF_LENGTH				(NSIZE)(			 10000)

/*-----------------------------------------------------------------------------------*/
/* Math Function Macro													             */
/*-----------------------------------------------------------------------------------*/
#define LOOP(Count,End)         for( Count=0; Count<End; Count++ )
#define GET_DIM(X)              (sizeof(X) / sizeof(X[0]))
#define SIGN_VAL(Value,Sign)    ((Sign >= 0.0) ? Value : -Value)

/*-----------------------------------------------------------------------------------*/
/* Common                                                                            */
/*-----------------------------------------------------------------------------------*/

enum	LogicFlag		        {NOK,YOK};
enum    IdxLidarData            {X_LID,Y_LID, NIDX_LIDARDATA};
enum    IdxLidarInt             {INT_LID, NIDX_LIDARINT};


/*-----------------------------------------------------------------------------------*/
/* Get Window Time (returns [msec])                                                  */
/*-----------------------------------------------------------------------------------*/

double	GetWindowTime	(void) ;

#endif