/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 164 */
real_t state[ 164 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 10];
state[1] = acadoVariables.x[lRun1 * 10 + 1];
state[2] = acadoVariables.x[lRun1 * 10 + 2];
state[3] = acadoVariables.x[lRun1 * 10 + 3];
state[4] = acadoVariables.x[lRun1 * 10 + 4];
state[5] = acadoVariables.x[lRun1 * 10 + 5];
state[6] = acadoVariables.x[lRun1 * 10 + 6];
state[7] = acadoVariables.x[lRun1 * 10 + 7];
state[8] = acadoVariables.x[lRun1 * 10 + 8];
state[9] = acadoVariables.x[lRun1 * 10 + 9];

state[150] = acadoVariables.u[lRun1 * 4];
state[151] = acadoVariables.u[lRun1 * 4 + 1];
state[152] = acadoVariables.u[lRun1 * 4 + 2];
state[153] = acadoVariables.u[lRun1 * 4 + 3];
state[154] = acadoVariables.od[lRun1 * 10];
state[155] = acadoVariables.od[lRun1 * 10 + 1];
state[156] = acadoVariables.od[lRun1 * 10 + 2];
state[157] = acadoVariables.od[lRun1 * 10 + 3];
state[158] = acadoVariables.od[lRun1 * 10 + 4];
state[159] = acadoVariables.od[lRun1 * 10 + 5];
state[160] = acadoVariables.od[lRun1 * 10 + 6];
state[161] = acadoVariables.od[lRun1 * 10 + 7];
state[162] = acadoVariables.od[lRun1 * 10 + 8];
state[163] = acadoVariables.od[lRun1 * 10 + 9];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 10] = state[0] - acadoVariables.x[lRun1 * 10 + 10];
acadoWorkspace.d[lRun1 * 10 + 1] = state[1] - acadoVariables.x[lRun1 * 10 + 11];
acadoWorkspace.d[lRun1 * 10 + 2] = state[2] - acadoVariables.x[lRun1 * 10 + 12];
acadoWorkspace.d[lRun1 * 10 + 3] = state[3] - acadoVariables.x[lRun1 * 10 + 13];
acadoWorkspace.d[lRun1 * 10 + 4] = state[4] - acadoVariables.x[lRun1 * 10 + 14];
acadoWorkspace.d[lRun1 * 10 + 5] = state[5] - acadoVariables.x[lRun1 * 10 + 15];
acadoWorkspace.d[lRun1 * 10 + 6] = state[6] - acadoVariables.x[lRun1 * 10 + 16];
acadoWorkspace.d[lRun1 * 10 + 7] = state[7] - acadoVariables.x[lRun1 * 10 + 17];
acadoWorkspace.d[lRun1 * 10 + 8] = state[8] - acadoVariables.x[lRun1 * 10 + 18];
acadoWorkspace.d[lRun1 * 10 + 9] = state[9] - acadoVariables.x[lRun1 * 10 + 19];

acadoWorkspace.evGx[lRun1 * 100] = state[10];
acadoWorkspace.evGx[lRun1 * 100 + 1] = state[11];
acadoWorkspace.evGx[lRun1 * 100 + 2] = state[12];
acadoWorkspace.evGx[lRun1 * 100 + 3] = state[13];
acadoWorkspace.evGx[lRun1 * 100 + 4] = state[14];
acadoWorkspace.evGx[lRun1 * 100 + 5] = state[15];
acadoWorkspace.evGx[lRun1 * 100 + 6] = state[16];
acadoWorkspace.evGx[lRun1 * 100 + 7] = state[17];
acadoWorkspace.evGx[lRun1 * 100 + 8] = state[18];
acadoWorkspace.evGx[lRun1 * 100 + 9] = state[19];
acadoWorkspace.evGx[lRun1 * 100 + 10] = state[20];
acadoWorkspace.evGx[lRun1 * 100 + 11] = state[21];
acadoWorkspace.evGx[lRun1 * 100 + 12] = state[22];
acadoWorkspace.evGx[lRun1 * 100 + 13] = state[23];
acadoWorkspace.evGx[lRun1 * 100 + 14] = state[24];
acadoWorkspace.evGx[lRun1 * 100 + 15] = state[25];
acadoWorkspace.evGx[lRun1 * 100 + 16] = state[26];
acadoWorkspace.evGx[lRun1 * 100 + 17] = state[27];
acadoWorkspace.evGx[lRun1 * 100 + 18] = state[28];
acadoWorkspace.evGx[lRun1 * 100 + 19] = state[29];
acadoWorkspace.evGx[lRun1 * 100 + 20] = state[30];
acadoWorkspace.evGx[lRun1 * 100 + 21] = state[31];
acadoWorkspace.evGx[lRun1 * 100 + 22] = state[32];
acadoWorkspace.evGx[lRun1 * 100 + 23] = state[33];
acadoWorkspace.evGx[lRun1 * 100 + 24] = state[34];
acadoWorkspace.evGx[lRun1 * 100 + 25] = state[35];
acadoWorkspace.evGx[lRun1 * 100 + 26] = state[36];
acadoWorkspace.evGx[lRun1 * 100 + 27] = state[37];
acadoWorkspace.evGx[lRun1 * 100 + 28] = state[38];
acadoWorkspace.evGx[lRun1 * 100 + 29] = state[39];
acadoWorkspace.evGx[lRun1 * 100 + 30] = state[40];
acadoWorkspace.evGx[lRun1 * 100 + 31] = state[41];
acadoWorkspace.evGx[lRun1 * 100 + 32] = state[42];
acadoWorkspace.evGx[lRun1 * 100 + 33] = state[43];
acadoWorkspace.evGx[lRun1 * 100 + 34] = state[44];
acadoWorkspace.evGx[lRun1 * 100 + 35] = state[45];
acadoWorkspace.evGx[lRun1 * 100 + 36] = state[46];
acadoWorkspace.evGx[lRun1 * 100 + 37] = state[47];
acadoWorkspace.evGx[lRun1 * 100 + 38] = state[48];
acadoWorkspace.evGx[lRun1 * 100 + 39] = state[49];
acadoWorkspace.evGx[lRun1 * 100 + 40] = state[50];
acadoWorkspace.evGx[lRun1 * 100 + 41] = state[51];
acadoWorkspace.evGx[lRun1 * 100 + 42] = state[52];
acadoWorkspace.evGx[lRun1 * 100 + 43] = state[53];
acadoWorkspace.evGx[lRun1 * 100 + 44] = state[54];
acadoWorkspace.evGx[lRun1 * 100 + 45] = state[55];
acadoWorkspace.evGx[lRun1 * 100 + 46] = state[56];
acadoWorkspace.evGx[lRun1 * 100 + 47] = state[57];
acadoWorkspace.evGx[lRun1 * 100 + 48] = state[58];
acadoWorkspace.evGx[lRun1 * 100 + 49] = state[59];
acadoWorkspace.evGx[lRun1 * 100 + 50] = state[60];
acadoWorkspace.evGx[lRun1 * 100 + 51] = state[61];
acadoWorkspace.evGx[lRun1 * 100 + 52] = state[62];
acadoWorkspace.evGx[lRun1 * 100 + 53] = state[63];
acadoWorkspace.evGx[lRun1 * 100 + 54] = state[64];
acadoWorkspace.evGx[lRun1 * 100 + 55] = state[65];
acadoWorkspace.evGx[lRun1 * 100 + 56] = state[66];
acadoWorkspace.evGx[lRun1 * 100 + 57] = state[67];
acadoWorkspace.evGx[lRun1 * 100 + 58] = state[68];
acadoWorkspace.evGx[lRun1 * 100 + 59] = state[69];
acadoWorkspace.evGx[lRun1 * 100 + 60] = state[70];
acadoWorkspace.evGx[lRun1 * 100 + 61] = state[71];
acadoWorkspace.evGx[lRun1 * 100 + 62] = state[72];
acadoWorkspace.evGx[lRun1 * 100 + 63] = state[73];
acadoWorkspace.evGx[lRun1 * 100 + 64] = state[74];
acadoWorkspace.evGx[lRun1 * 100 + 65] = state[75];
acadoWorkspace.evGx[lRun1 * 100 + 66] = state[76];
acadoWorkspace.evGx[lRun1 * 100 + 67] = state[77];
acadoWorkspace.evGx[lRun1 * 100 + 68] = state[78];
acadoWorkspace.evGx[lRun1 * 100 + 69] = state[79];
acadoWorkspace.evGx[lRun1 * 100 + 70] = state[80];
acadoWorkspace.evGx[lRun1 * 100 + 71] = state[81];
acadoWorkspace.evGx[lRun1 * 100 + 72] = state[82];
acadoWorkspace.evGx[lRun1 * 100 + 73] = state[83];
acadoWorkspace.evGx[lRun1 * 100 + 74] = state[84];
acadoWorkspace.evGx[lRun1 * 100 + 75] = state[85];
acadoWorkspace.evGx[lRun1 * 100 + 76] = state[86];
acadoWorkspace.evGx[lRun1 * 100 + 77] = state[87];
acadoWorkspace.evGx[lRun1 * 100 + 78] = state[88];
acadoWorkspace.evGx[lRun1 * 100 + 79] = state[89];
acadoWorkspace.evGx[lRun1 * 100 + 80] = state[90];
acadoWorkspace.evGx[lRun1 * 100 + 81] = state[91];
acadoWorkspace.evGx[lRun1 * 100 + 82] = state[92];
acadoWorkspace.evGx[lRun1 * 100 + 83] = state[93];
acadoWorkspace.evGx[lRun1 * 100 + 84] = state[94];
acadoWorkspace.evGx[lRun1 * 100 + 85] = state[95];
acadoWorkspace.evGx[lRun1 * 100 + 86] = state[96];
acadoWorkspace.evGx[lRun1 * 100 + 87] = state[97];
acadoWorkspace.evGx[lRun1 * 100 + 88] = state[98];
acadoWorkspace.evGx[lRun1 * 100 + 89] = state[99];
acadoWorkspace.evGx[lRun1 * 100 + 90] = state[100];
acadoWorkspace.evGx[lRun1 * 100 + 91] = state[101];
acadoWorkspace.evGx[lRun1 * 100 + 92] = state[102];
acadoWorkspace.evGx[lRun1 * 100 + 93] = state[103];
acadoWorkspace.evGx[lRun1 * 100 + 94] = state[104];
acadoWorkspace.evGx[lRun1 * 100 + 95] = state[105];
acadoWorkspace.evGx[lRun1 * 100 + 96] = state[106];
acadoWorkspace.evGx[lRun1 * 100 + 97] = state[107];
acadoWorkspace.evGx[lRun1 * 100 + 98] = state[108];
acadoWorkspace.evGx[lRun1 * 100 + 99] = state[109];

acadoWorkspace.evGu[lRun1 * 40] = state[110];
acadoWorkspace.evGu[lRun1 * 40 + 1] = state[111];
acadoWorkspace.evGu[lRun1 * 40 + 2] = state[112];
acadoWorkspace.evGu[lRun1 * 40 + 3] = state[113];
acadoWorkspace.evGu[lRun1 * 40 + 4] = state[114];
acadoWorkspace.evGu[lRun1 * 40 + 5] = state[115];
acadoWorkspace.evGu[lRun1 * 40 + 6] = state[116];
acadoWorkspace.evGu[lRun1 * 40 + 7] = state[117];
acadoWorkspace.evGu[lRun1 * 40 + 8] = state[118];
acadoWorkspace.evGu[lRun1 * 40 + 9] = state[119];
acadoWorkspace.evGu[lRun1 * 40 + 10] = state[120];
acadoWorkspace.evGu[lRun1 * 40 + 11] = state[121];
acadoWorkspace.evGu[lRun1 * 40 + 12] = state[122];
acadoWorkspace.evGu[lRun1 * 40 + 13] = state[123];
acadoWorkspace.evGu[lRun1 * 40 + 14] = state[124];
acadoWorkspace.evGu[lRun1 * 40 + 15] = state[125];
acadoWorkspace.evGu[lRun1 * 40 + 16] = state[126];
acadoWorkspace.evGu[lRun1 * 40 + 17] = state[127];
acadoWorkspace.evGu[lRun1 * 40 + 18] = state[128];
acadoWorkspace.evGu[lRun1 * 40 + 19] = state[129];
acadoWorkspace.evGu[lRun1 * 40 + 20] = state[130];
acadoWorkspace.evGu[lRun1 * 40 + 21] = state[131];
acadoWorkspace.evGu[lRun1 * 40 + 22] = state[132];
acadoWorkspace.evGu[lRun1 * 40 + 23] = state[133];
acadoWorkspace.evGu[lRun1 * 40 + 24] = state[134];
acadoWorkspace.evGu[lRun1 * 40 + 25] = state[135];
acadoWorkspace.evGu[lRun1 * 40 + 26] = state[136];
acadoWorkspace.evGu[lRun1 * 40 + 27] = state[137];
acadoWorkspace.evGu[lRun1 * 40 + 28] = state[138];
acadoWorkspace.evGu[lRun1 * 40 + 29] = state[139];
acadoWorkspace.evGu[lRun1 * 40 + 30] = state[140];
acadoWorkspace.evGu[lRun1 * 40 + 31] = state[141];
acadoWorkspace.evGu[lRun1 * 40 + 32] = state[142];
acadoWorkspace.evGu[lRun1 * 40 + 33] = state[143];
acadoWorkspace.evGu[lRun1 * 40 + 34] = state[144];
acadoWorkspace.evGu[lRun1 * 40 + 35] = state[145];
acadoWorkspace.evGu[lRun1 * 40 + 36] = state[146];
acadoWorkspace.evGu[lRun1 * 40 + 37] = state[147];
acadoWorkspace.evGu[lRun1 * 40 + 38] = state[148];
acadoWorkspace.evGu[lRun1 * 40 + 39] = state[149];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = u[0];
out[11] = u[1];
out[12] = u[2];
out[13] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;
/* Vector of auxiliary variables; number of elements: 21. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = ((u[1])*(u[1]));
a[1] = ((u[2])*(u[2]));
a[2] = (sqrt(((a[0]+a[1])+(real_t)(1.0000000000000000e-04))));
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(1.0000000000000000e+00);
a[14] = ((real_t)(2.0000000000000000e+00)*u[1]);
a[15] = (1.0/sqrt(((a[0]+a[1])+(real_t)(1.0000000000000000e-04))));
a[16] = (a[15]*(real_t)(5.0000000000000000e-01));
a[17] = (a[14]*a[16]);
a[18] = ((real_t)(2.0000000000000000e+00)*u[2]);
a[19] = (a[18]*a[16]);
a[20] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (u[0]+a[2]);
out[1] = a[3];
out[2] = a[4];
out[3] = a[5];
out[4] = a[6];
out[5] = a[7];
out[6] = a[8];
out[7] = a[9];
out[8] = a[10];
out[9] = a[11];
out[10] = a[12];
out[11] = a[13];
out[12] = a[17];
out[13] = a[19];
out[14] = a[20];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = +tmpObjS[60];
tmpQ2[61] = +tmpObjS[61];
tmpQ2[62] = +tmpObjS[62];
tmpQ2[63] = +tmpObjS[63];
tmpQ2[64] = +tmpObjS[64];
tmpQ2[65] = +tmpObjS[65];
tmpQ2[66] = +tmpObjS[66];
tmpQ2[67] = +tmpObjS[67];
tmpQ2[68] = +tmpObjS[68];
tmpQ2[69] = +tmpObjS[69];
tmpQ2[70] = +tmpObjS[70];
tmpQ2[71] = +tmpObjS[71];
tmpQ2[72] = +tmpObjS[72];
tmpQ2[73] = +tmpObjS[73];
tmpQ2[74] = +tmpObjS[74];
tmpQ2[75] = +tmpObjS[75];
tmpQ2[76] = +tmpObjS[76];
tmpQ2[77] = +tmpObjS[77];
tmpQ2[78] = +tmpObjS[78];
tmpQ2[79] = +tmpObjS[79];
tmpQ2[80] = +tmpObjS[80];
tmpQ2[81] = +tmpObjS[81];
tmpQ2[82] = +tmpObjS[82];
tmpQ2[83] = +tmpObjS[83];
tmpQ2[84] = +tmpObjS[84];
tmpQ2[85] = +tmpObjS[85];
tmpQ2[86] = +tmpObjS[86];
tmpQ2[87] = +tmpObjS[87];
tmpQ2[88] = +tmpObjS[88];
tmpQ2[89] = +tmpObjS[89];
tmpQ2[90] = +tmpObjS[90];
tmpQ2[91] = +tmpObjS[91];
tmpQ2[92] = +tmpObjS[92];
tmpQ2[93] = +tmpObjS[93];
tmpQ2[94] = +tmpObjS[94];
tmpQ2[95] = +tmpObjS[95];
tmpQ2[96] = +tmpObjS[96];
tmpQ2[97] = +tmpObjS[97];
tmpQ2[98] = +tmpObjS[98];
tmpQ2[99] = +tmpObjS[99];
tmpQ2[100] = +tmpObjS[100];
tmpQ2[101] = +tmpObjS[101];
tmpQ2[102] = +tmpObjS[102];
tmpQ2[103] = +tmpObjS[103];
tmpQ2[104] = +tmpObjS[104];
tmpQ2[105] = +tmpObjS[105];
tmpQ2[106] = +tmpObjS[106];
tmpQ2[107] = +tmpObjS[107];
tmpQ2[108] = +tmpObjS[108];
tmpQ2[109] = +tmpObjS[109];
tmpQ2[110] = +tmpObjS[110];
tmpQ2[111] = +tmpObjS[111];
tmpQ2[112] = +tmpObjS[112];
tmpQ2[113] = +tmpObjS[113];
tmpQ2[114] = +tmpObjS[114];
tmpQ2[115] = +tmpObjS[115];
tmpQ2[116] = +tmpObjS[116];
tmpQ2[117] = +tmpObjS[117];
tmpQ2[118] = +tmpObjS[118];
tmpQ2[119] = +tmpObjS[119];
tmpQ2[120] = +tmpObjS[120];
tmpQ2[121] = +tmpObjS[121];
tmpQ2[122] = +tmpObjS[122];
tmpQ2[123] = +tmpObjS[123];
tmpQ2[124] = +tmpObjS[124];
tmpQ2[125] = +tmpObjS[125];
tmpQ2[126] = +tmpObjS[126];
tmpQ2[127] = +tmpObjS[127];
tmpQ2[128] = +tmpObjS[128];
tmpQ2[129] = +tmpObjS[129];
tmpQ2[130] = +tmpObjS[130];
tmpQ2[131] = +tmpObjS[131];
tmpQ2[132] = +tmpObjS[132];
tmpQ2[133] = +tmpObjS[133];
tmpQ2[134] = +tmpObjS[134];
tmpQ2[135] = +tmpObjS[135];
tmpQ2[136] = +tmpObjS[136];
tmpQ2[137] = +tmpObjS[137];
tmpQ2[138] = +tmpObjS[138];
tmpQ2[139] = +tmpObjS[139];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[9];
tmpQ1[10] = + tmpQ2[14];
tmpQ1[11] = + tmpQ2[15];
tmpQ1[12] = + tmpQ2[16];
tmpQ1[13] = + tmpQ2[17];
tmpQ1[14] = + tmpQ2[18];
tmpQ1[15] = + tmpQ2[19];
tmpQ1[16] = + tmpQ2[20];
tmpQ1[17] = + tmpQ2[21];
tmpQ1[18] = + tmpQ2[22];
tmpQ1[19] = + tmpQ2[23];
tmpQ1[20] = + tmpQ2[28];
tmpQ1[21] = + tmpQ2[29];
tmpQ1[22] = + tmpQ2[30];
tmpQ1[23] = + tmpQ2[31];
tmpQ1[24] = + tmpQ2[32];
tmpQ1[25] = + tmpQ2[33];
tmpQ1[26] = + tmpQ2[34];
tmpQ1[27] = + tmpQ2[35];
tmpQ1[28] = + tmpQ2[36];
tmpQ1[29] = + tmpQ2[37];
tmpQ1[30] = + tmpQ2[42];
tmpQ1[31] = + tmpQ2[43];
tmpQ1[32] = + tmpQ2[44];
tmpQ1[33] = + tmpQ2[45];
tmpQ1[34] = + tmpQ2[46];
tmpQ1[35] = + tmpQ2[47];
tmpQ1[36] = + tmpQ2[48];
tmpQ1[37] = + tmpQ2[49];
tmpQ1[38] = + tmpQ2[50];
tmpQ1[39] = + tmpQ2[51];
tmpQ1[40] = + tmpQ2[56];
tmpQ1[41] = + tmpQ2[57];
tmpQ1[42] = + tmpQ2[58];
tmpQ1[43] = + tmpQ2[59];
tmpQ1[44] = + tmpQ2[60];
tmpQ1[45] = + tmpQ2[61];
tmpQ1[46] = + tmpQ2[62];
tmpQ1[47] = + tmpQ2[63];
tmpQ1[48] = + tmpQ2[64];
tmpQ1[49] = + tmpQ2[65];
tmpQ1[50] = + tmpQ2[70];
tmpQ1[51] = + tmpQ2[71];
tmpQ1[52] = + tmpQ2[72];
tmpQ1[53] = + tmpQ2[73];
tmpQ1[54] = + tmpQ2[74];
tmpQ1[55] = + tmpQ2[75];
tmpQ1[56] = + tmpQ2[76];
tmpQ1[57] = + tmpQ2[77];
tmpQ1[58] = + tmpQ2[78];
tmpQ1[59] = + tmpQ2[79];
tmpQ1[60] = + tmpQ2[84];
tmpQ1[61] = + tmpQ2[85];
tmpQ1[62] = + tmpQ2[86];
tmpQ1[63] = + tmpQ2[87];
tmpQ1[64] = + tmpQ2[88];
tmpQ1[65] = + tmpQ2[89];
tmpQ1[66] = + tmpQ2[90];
tmpQ1[67] = + tmpQ2[91];
tmpQ1[68] = + tmpQ2[92];
tmpQ1[69] = + tmpQ2[93];
tmpQ1[70] = + tmpQ2[98];
tmpQ1[71] = + tmpQ2[99];
tmpQ1[72] = + tmpQ2[100];
tmpQ1[73] = + tmpQ2[101];
tmpQ1[74] = + tmpQ2[102];
tmpQ1[75] = + tmpQ2[103];
tmpQ1[76] = + tmpQ2[104];
tmpQ1[77] = + tmpQ2[105];
tmpQ1[78] = + tmpQ2[106];
tmpQ1[79] = + tmpQ2[107];
tmpQ1[80] = + tmpQ2[112];
tmpQ1[81] = + tmpQ2[113];
tmpQ1[82] = + tmpQ2[114];
tmpQ1[83] = + tmpQ2[115];
tmpQ1[84] = + tmpQ2[116];
tmpQ1[85] = + tmpQ2[117];
tmpQ1[86] = + tmpQ2[118];
tmpQ1[87] = + tmpQ2[119];
tmpQ1[88] = + tmpQ2[120];
tmpQ1[89] = + tmpQ2[121];
tmpQ1[90] = + tmpQ2[126];
tmpQ1[91] = + tmpQ2[127];
tmpQ1[92] = + tmpQ2[128];
tmpQ1[93] = + tmpQ2[129];
tmpQ1[94] = + tmpQ2[130];
tmpQ1[95] = + tmpQ2[131];
tmpQ1[96] = + tmpQ2[132];
tmpQ1[97] = + tmpQ2[133];
tmpQ1[98] = + tmpQ2[134];
tmpQ1[99] = + tmpQ2[135];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[140];
tmpR2[1] = +tmpObjS[141];
tmpR2[2] = +tmpObjS[142];
tmpR2[3] = +tmpObjS[143];
tmpR2[4] = +tmpObjS[144];
tmpR2[5] = +tmpObjS[145];
tmpR2[6] = +tmpObjS[146];
tmpR2[7] = +tmpObjS[147];
tmpR2[8] = +tmpObjS[148];
tmpR2[9] = +tmpObjS[149];
tmpR2[10] = +tmpObjS[150];
tmpR2[11] = +tmpObjS[151];
tmpR2[12] = +tmpObjS[152];
tmpR2[13] = +tmpObjS[153];
tmpR2[14] = +tmpObjS[154];
tmpR2[15] = +tmpObjS[155];
tmpR2[16] = +tmpObjS[156];
tmpR2[17] = +tmpObjS[157];
tmpR2[18] = +tmpObjS[158];
tmpR2[19] = +tmpObjS[159];
tmpR2[20] = +tmpObjS[160];
tmpR2[21] = +tmpObjS[161];
tmpR2[22] = +tmpObjS[162];
tmpR2[23] = +tmpObjS[163];
tmpR2[24] = +tmpObjS[164];
tmpR2[25] = +tmpObjS[165];
tmpR2[26] = +tmpObjS[166];
tmpR2[27] = +tmpObjS[167];
tmpR2[28] = +tmpObjS[168];
tmpR2[29] = +tmpObjS[169];
tmpR2[30] = +tmpObjS[170];
tmpR2[31] = +tmpObjS[171];
tmpR2[32] = +tmpObjS[172];
tmpR2[33] = +tmpObjS[173];
tmpR2[34] = +tmpObjS[174];
tmpR2[35] = +tmpObjS[175];
tmpR2[36] = +tmpObjS[176];
tmpR2[37] = +tmpObjS[177];
tmpR2[38] = +tmpObjS[178];
tmpR2[39] = +tmpObjS[179];
tmpR2[40] = +tmpObjS[180];
tmpR2[41] = +tmpObjS[181];
tmpR2[42] = +tmpObjS[182];
tmpR2[43] = +tmpObjS[183];
tmpR2[44] = +tmpObjS[184];
tmpR2[45] = +tmpObjS[185];
tmpR2[46] = +tmpObjS[186];
tmpR2[47] = +tmpObjS[187];
tmpR2[48] = +tmpObjS[188];
tmpR2[49] = +tmpObjS[189];
tmpR2[50] = +tmpObjS[190];
tmpR2[51] = +tmpObjS[191];
tmpR2[52] = +tmpObjS[192];
tmpR2[53] = +tmpObjS[193];
tmpR2[54] = +tmpObjS[194];
tmpR2[55] = +tmpObjS[195];
tmpR1[0] = + tmpR2[10];
tmpR1[1] = + tmpR2[11];
tmpR1[2] = + tmpR2[12];
tmpR1[3] = + tmpR2[13];
tmpR1[4] = + tmpR2[24];
tmpR1[5] = + tmpR2[25];
tmpR1[6] = + tmpR2[26];
tmpR1[7] = + tmpR2[27];
tmpR1[8] = + tmpR2[38];
tmpR1[9] = + tmpR2[39];
tmpR1[10] = + tmpR2[40];
tmpR1[11] = + tmpR2[41];
tmpR1[12] = + tmpR2[52];
tmpR1[13] = + tmpR2[53];
tmpR1[14] = + tmpR2[54];
tmpR1[15] = + tmpR2[55];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN2[64] = +tmpObjSEndTerm[64];
tmpQN2[65] = +tmpObjSEndTerm[65];
tmpQN2[66] = +tmpObjSEndTerm[66];
tmpQN2[67] = +tmpObjSEndTerm[67];
tmpQN2[68] = +tmpObjSEndTerm[68];
tmpQN2[69] = +tmpObjSEndTerm[69];
tmpQN2[70] = +tmpObjSEndTerm[70];
tmpQN2[71] = +tmpObjSEndTerm[71];
tmpQN2[72] = +tmpObjSEndTerm[72];
tmpQN2[73] = +tmpObjSEndTerm[73];
tmpQN2[74] = +tmpObjSEndTerm[74];
tmpQN2[75] = +tmpObjSEndTerm[75];
tmpQN2[76] = +tmpObjSEndTerm[76];
tmpQN2[77] = +tmpObjSEndTerm[77];
tmpQN2[78] = +tmpObjSEndTerm[78];
tmpQN2[79] = +tmpObjSEndTerm[79];
tmpQN2[80] = +tmpObjSEndTerm[80];
tmpQN2[81] = +tmpObjSEndTerm[81];
tmpQN2[82] = +tmpObjSEndTerm[82];
tmpQN2[83] = +tmpObjSEndTerm[83];
tmpQN2[84] = +tmpObjSEndTerm[84];
tmpQN2[85] = +tmpObjSEndTerm[85];
tmpQN2[86] = +tmpObjSEndTerm[86];
tmpQN2[87] = +tmpObjSEndTerm[87];
tmpQN2[88] = +tmpObjSEndTerm[88];
tmpQN2[89] = +tmpObjSEndTerm[89];
tmpQN2[90] = +tmpObjSEndTerm[90];
tmpQN2[91] = +tmpObjSEndTerm[91];
tmpQN2[92] = +tmpObjSEndTerm[92];
tmpQN2[93] = +tmpObjSEndTerm[93];
tmpQN2[94] = +tmpObjSEndTerm[94];
tmpQN2[95] = +tmpObjSEndTerm[95];
tmpQN2[96] = +tmpObjSEndTerm[96];
tmpQN2[97] = +tmpObjSEndTerm[97];
tmpQN2[98] = +tmpObjSEndTerm[98];
tmpQN2[99] = +tmpObjSEndTerm[99];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
tmpQN1[49] = + tmpQN2[49];
tmpQN1[50] = + tmpQN2[50];
tmpQN1[51] = + tmpQN2[51];
tmpQN1[52] = + tmpQN2[52];
tmpQN1[53] = + tmpQN2[53];
tmpQN1[54] = + tmpQN2[54];
tmpQN1[55] = + tmpQN2[55];
tmpQN1[56] = + tmpQN2[56];
tmpQN1[57] = + tmpQN2[57];
tmpQN1[58] = + tmpQN2[58];
tmpQN1[59] = + tmpQN2[59];
tmpQN1[60] = + tmpQN2[60];
tmpQN1[61] = + tmpQN2[61];
tmpQN1[62] = + tmpQN2[62];
tmpQN1[63] = + tmpQN2[63];
tmpQN1[64] = + tmpQN2[64];
tmpQN1[65] = + tmpQN2[65];
tmpQN1[66] = + tmpQN2[66];
tmpQN1[67] = + tmpQN2[67];
tmpQN1[68] = + tmpQN2[68];
tmpQN1[69] = + tmpQN2[69];
tmpQN1[70] = + tmpQN2[70];
tmpQN1[71] = + tmpQN2[71];
tmpQN1[72] = + tmpQN2[72];
tmpQN1[73] = + tmpQN2[73];
tmpQN1[74] = + tmpQN2[74];
tmpQN1[75] = + tmpQN2[75];
tmpQN1[76] = + tmpQN2[76];
tmpQN1[77] = + tmpQN2[77];
tmpQN1[78] = + tmpQN2[78];
tmpQN1[79] = + tmpQN2[79];
tmpQN1[80] = + tmpQN2[80];
tmpQN1[81] = + tmpQN2[81];
tmpQN1[82] = + tmpQN2[82];
tmpQN1[83] = + tmpQN2[83];
tmpQN1[84] = + tmpQN2[84];
tmpQN1[85] = + tmpQN2[85];
tmpQN1[86] = + tmpQN2[86];
tmpQN1[87] = + tmpQN2[87];
tmpQN1[88] = + tmpQN2[88];
tmpQN1[89] = + tmpQN2[89];
tmpQN1[90] = + tmpQN2[90];
tmpQN1[91] = + tmpQN2[91];
tmpQN1[92] = + tmpQN2[92];
tmpQN1[93] = + tmpQN2[93];
tmpQN1[94] = + tmpQN2[94];
tmpQN1[95] = + tmpQN2[95];
tmpQN1[96] = + tmpQN2[96];
tmpQN1[97] = + tmpQN2[97];
tmpQN1[98] = + tmpQN2[98];
tmpQN1[99] = + tmpQN2[99];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4 + 3];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 10];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 10 + 1];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 10 + 2];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 10 + 3];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 10 + 4];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 10 + 5];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 10 + 6];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 10 + 7];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 10 + 8];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 10 + 9];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 14] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 14 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 14 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 14 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 14 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 14 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 14 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 14 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 14 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 14 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 14 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 14 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 14 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 14 + 13] = acadoWorkspace.objValueOut[13];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 196 ]), &(acadoWorkspace.Q1[ runObj * 100 ]), &(acadoWorkspace.Q2[ runObj * 140 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 196 ]), &(acadoWorkspace.R1[ runObj * 16 ]), &(acadoWorkspace.R2[ runObj * 56 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[200];
acadoWorkspace.objValueIn[1] = acadoVariables.x[201];
acadoWorkspace.objValueIn[2] = acadoVariables.x[202];
acadoWorkspace.objValueIn[3] = acadoVariables.x[203];
acadoWorkspace.objValueIn[4] = acadoVariables.x[204];
acadoWorkspace.objValueIn[5] = acadoVariables.x[205];
acadoWorkspace.objValueIn[6] = acadoVariables.x[206];
acadoWorkspace.objValueIn[7] = acadoVariables.x[207];
acadoWorkspace.objValueIn[8] = acadoVariables.x[208];
acadoWorkspace.objValueIn[9] = acadoVariables.x[209];
acadoWorkspace.objValueIn[10] = acadoVariables.od[200];
acadoWorkspace.objValueIn[11] = acadoVariables.od[201];
acadoWorkspace.objValueIn[12] = acadoVariables.od[202];
acadoWorkspace.objValueIn[13] = acadoVariables.od[203];
acadoWorkspace.objValueIn[14] = acadoVariables.od[204];
acadoWorkspace.objValueIn[15] = acadoVariables.od[205];
acadoWorkspace.objValueIn[16] = acadoVariables.od[206];
acadoWorkspace.objValueIn[17] = acadoVariables.od[207];
acadoWorkspace.objValueIn[18] = acadoVariables.od[208];
acadoWorkspace.objValueIn[19] = acadoVariables.od[209];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
Gx2[64] = Gx1[64];
Gx2[65] = Gx1[65];
Gx2[66] = Gx1[66];
Gx2[67] = Gx1[67];
Gx2[68] = Gx1[68];
Gx2[69] = Gx1[69];
Gx2[70] = Gx1[70];
Gx2[71] = Gx1[71];
Gx2[72] = Gx1[72];
Gx2[73] = Gx1[73];
Gx2[74] = Gx1[74];
Gx2[75] = Gx1[75];
Gx2[76] = Gx1[76];
Gx2[77] = Gx1[77];
Gx2[78] = Gx1[78];
Gx2[79] = Gx1[79];
Gx2[80] = Gx1[80];
Gx2[81] = Gx1[81];
Gx2[82] = Gx1[82];
Gx2[83] = Gx1[83];
Gx2[84] = Gx1[84];
Gx2[85] = Gx1[85];
Gx2[86] = Gx1[86];
Gx2[87] = Gx1[87];
Gx2[88] = Gx1[88];
Gx2[89] = Gx1[89];
Gx2[90] = Gx1[90];
Gx2[91] = Gx1[91];
Gx2[92] = Gx1[92];
Gx2[93] = Gx1[93];
Gx2[94] = Gx1[94];
Gx2[95] = Gx1[95];
Gx2[96] = Gx1[96];
Gx2[97] = Gx1[97];
Gx2[98] = Gx1[98];
Gx2[99] = Gx1[99];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[40] + Gx1[5]*Gx2[50] + Gx1[6]*Gx2[60] + Gx1[7]*Gx2[70] + Gx1[8]*Gx2[80] + Gx1[9]*Gx2[90];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[41] + Gx1[5]*Gx2[51] + Gx1[6]*Gx2[61] + Gx1[7]*Gx2[71] + Gx1[8]*Gx2[81] + Gx1[9]*Gx2[91];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[32] + Gx1[4]*Gx2[42] + Gx1[5]*Gx2[52] + Gx1[6]*Gx2[62] + Gx1[7]*Gx2[72] + Gx1[8]*Gx2[82] + Gx1[9]*Gx2[92];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[33] + Gx1[4]*Gx2[43] + Gx1[5]*Gx2[53] + Gx1[6]*Gx2[63] + Gx1[7]*Gx2[73] + Gx1[8]*Gx2[83] + Gx1[9]*Gx2[93];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[34] + Gx1[4]*Gx2[44] + Gx1[5]*Gx2[54] + Gx1[6]*Gx2[64] + Gx1[7]*Gx2[74] + Gx1[8]*Gx2[84] + Gx1[9]*Gx2[94];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[35] + Gx1[4]*Gx2[45] + Gx1[5]*Gx2[55] + Gx1[6]*Gx2[65] + Gx1[7]*Gx2[75] + Gx1[8]*Gx2[85] + Gx1[9]*Gx2[95];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[36] + Gx1[4]*Gx2[46] + Gx1[5]*Gx2[56] + Gx1[6]*Gx2[66] + Gx1[7]*Gx2[76] + Gx1[8]*Gx2[86] + Gx1[9]*Gx2[96];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[27] + Gx1[3]*Gx2[37] + Gx1[4]*Gx2[47] + Gx1[5]*Gx2[57] + Gx1[6]*Gx2[67] + Gx1[7]*Gx2[77] + Gx1[8]*Gx2[87] + Gx1[9]*Gx2[97];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[18] + Gx1[2]*Gx2[28] + Gx1[3]*Gx2[38] + Gx1[4]*Gx2[48] + Gx1[5]*Gx2[58] + Gx1[6]*Gx2[68] + Gx1[7]*Gx2[78] + Gx1[8]*Gx2[88] + Gx1[9]*Gx2[98];
Gx3[9] = + Gx1[0]*Gx2[9] + Gx1[1]*Gx2[19] + Gx1[2]*Gx2[29] + Gx1[3]*Gx2[39] + Gx1[4]*Gx2[49] + Gx1[5]*Gx2[59] + Gx1[6]*Gx2[69] + Gx1[7]*Gx2[79] + Gx1[8]*Gx2[89] + Gx1[9]*Gx2[99];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[10] + Gx1[12]*Gx2[20] + Gx1[13]*Gx2[30] + Gx1[14]*Gx2[40] + Gx1[15]*Gx2[50] + Gx1[16]*Gx2[60] + Gx1[17]*Gx2[70] + Gx1[18]*Gx2[80] + Gx1[19]*Gx2[90];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[11] + Gx1[12]*Gx2[21] + Gx1[13]*Gx2[31] + Gx1[14]*Gx2[41] + Gx1[15]*Gx2[51] + Gx1[16]*Gx2[61] + Gx1[17]*Gx2[71] + Gx1[18]*Gx2[81] + Gx1[19]*Gx2[91];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[12] + Gx1[12]*Gx2[22] + Gx1[13]*Gx2[32] + Gx1[14]*Gx2[42] + Gx1[15]*Gx2[52] + Gx1[16]*Gx2[62] + Gx1[17]*Gx2[72] + Gx1[18]*Gx2[82] + Gx1[19]*Gx2[92];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[13] + Gx1[12]*Gx2[23] + Gx1[13]*Gx2[33] + Gx1[14]*Gx2[43] + Gx1[15]*Gx2[53] + Gx1[16]*Gx2[63] + Gx1[17]*Gx2[73] + Gx1[18]*Gx2[83] + Gx1[19]*Gx2[93];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[14] + Gx1[12]*Gx2[24] + Gx1[13]*Gx2[34] + Gx1[14]*Gx2[44] + Gx1[15]*Gx2[54] + Gx1[16]*Gx2[64] + Gx1[17]*Gx2[74] + Gx1[18]*Gx2[84] + Gx1[19]*Gx2[94];
Gx3[15] = + Gx1[10]*Gx2[5] + Gx1[11]*Gx2[15] + Gx1[12]*Gx2[25] + Gx1[13]*Gx2[35] + Gx1[14]*Gx2[45] + Gx1[15]*Gx2[55] + Gx1[16]*Gx2[65] + Gx1[17]*Gx2[75] + Gx1[18]*Gx2[85] + Gx1[19]*Gx2[95];
Gx3[16] = + Gx1[10]*Gx2[6] + Gx1[11]*Gx2[16] + Gx1[12]*Gx2[26] + Gx1[13]*Gx2[36] + Gx1[14]*Gx2[46] + Gx1[15]*Gx2[56] + Gx1[16]*Gx2[66] + Gx1[17]*Gx2[76] + Gx1[18]*Gx2[86] + Gx1[19]*Gx2[96];
Gx3[17] = + Gx1[10]*Gx2[7] + Gx1[11]*Gx2[17] + Gx1[12]*Gx2[27] + Gx1[13]*Gx2[37] + Gx1[14]*Gx2[47] + Gx1[15]*Gx2[57] + Gx1[16]*Gx2[67] + Gx1[17]*Gx2[77] + Gx1[18]*Gx2[87] + Gx1[19]*Gx2[97];
Gx3[18] = + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[18] + Gx1[12]*Gx2[28] + Gx1[13]*Gx2[38] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[58] + Gx1[16]*Gx2[68] + Gx1[17]*Gx2[78] + Gx1[18]*Gx2[88] + Gx1[19]*Gx2[98];
Gx3[19] = + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[19] + Gx1[12]*Gx2[29] + Gx1[13]*Gx2[39] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[59] + Gx1[16]*Gx2[69] + Gx1[17]*Gx2[79] + Gx1[18]*Gx2[89] + Gx1[19]*Gx2[99];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[10] + Gx1[22]*Gx2[20] + Gx1[23]*Gx2[30] + Gx1[24]*Gx2[40] + Gx1[25]*Gx2[50] + Gx1[26]*Gx2[60] + Gx1[27]*Gx2[70] + Gx1[28]*Gx2[80] + Gx1[29]*Gx2[90];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[11] + Gx1[22]*Gx2[21] + Gx1[23]*Gx2[31] + Gx1[24]*Gx2[41] + Gx1[25]*Gx2[51] + Gx1[26]*Gx2[61] + Gx1[27]*Gx2[71] + Gx1[28]*Gx2[81] + Gx1[29]*Gx2[91];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[12] + Gx1[22]*Gx2[22] + Gx1[23]*Gx2[32] + Gx1[24]*Gx2[42] + Gx1[25]*Gx2[52] + Gx1[26]*Gx2[62] + Gx1[27]*Gx2[72] + Gx1[28]*Gx2[82] + Gx1[29]*Gx2[92];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[13] + Gx1[22]*Gx2[23] + Gx1[23]*Gx2[33] + Gx1[24]*Gx2[43] + Gx1[25]*Gx2[53] + Gx1[26]*Gx2[63] + Gx1[27]*Gx2[73] + Gx1[28]*Gx2[83] + Gx1[29]*Gx2[93];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[14] + Gx1[22]*Gx2[24] + Gx1[23]*Gx2[34] + Gx1[24]*Gx2[44] + Gx1[25]*Gx2[54] + Gx1[26]*Gx2[64] + Gx1[27]*Gx2[74] + Gx1[28]*Gx2[84] + Gx1[29]*Gx2[94];
Gx3[25] = + Gx1[20]*Gx2[5] + Gx1[21]*Gx2[15] + Gx1[22]*Gx2[25] + Gx1[23]*Gx2[35] + Gx1[24]*Gx2[45] + Gx1[25]*Gx2[55] + Gx1[26]*Gx2[65] + Gx1[27]*Gx2[75] + Gx1[28]*Gx2[85] + Gx1[29]*Gx2[95];
Gx3[26] = + Gx1[20]*Gx2[6] + Gx1[21]*Gx2[16] + Gx1[22]*Gx2[26] + Gx1[23]*Gx2[36] + Gx1[24]*Gx2[46] + Gx1[25]*Gx2[56] + Gx1[26]*Gx2[66] + Gx1[27]*Gx2[76] + Gx1[28]*Gx2[86] + Gx1[29]*Gx2[96];
Gx3[27] = + Gx1[20]*Gx2[7] + Gx1[21]*Gx2[17] + Gx1[22]*Gx2[27] + Gx1[23]*Gx2[37] + Gx1[24]*Gx2[47] + Gx1[25]*Gx2[57] + Gx1[26]*Gx2[67] + Gx1[27]*Gx2[77] + Gx1[28]*Gx2[87] + Gx1[29]*Gx2[97];
Gx3[28] = + Gx1[20]*Gx2[8] + Gx1[21]*Gx2[18] + Gx1[22]*Gx2[28] + Gx1[23]*Gx2[38] + Gx1[24]*Gx2[48] + Gx1[25]*Gx2[58] + Gx1[26]*Gx2[68] + Gx1[27]*Gx2[78] + Gx1[28]*Gx2[88] + Gx1[29]*Gx2[98];
Gx3[29] = + Gx1[20]*Gx2[9] + Gx1[21]*Gx2[19] + Gx1[22]*Gx2[29] + Gx1[23]*Gx2[39] + Gx1[24]*Gx2[49] + Gx1[25]*Gx2[59] + Gx1[26]*Gx2[69] + Gx1[27]*Gx2[79] + Gx1[28]*Gx2[89] + Gx1[29]*Gx2[99];
Gx3[30] = + Gx1[30]*Gx2[0] + Gx1[31]*Gx2[10] + Gx1[32]*Gx2[20] + Gx1[33]*Gx2[30] + Gx1[34]*Gx2[40] + Gx1[35]*Gx2[50] + Gx1[36]*Gx2[60] + Gx1[37]*Gx2[70] + Gx1[38]*Gx2[80] + Gx1[39]*Gx2[90];
Gx3[31] = + Gx1[30]*Gx2[1] + Gx1[31]*Gx2[11] + Gx1[32]*Gx2[21] + Gx1[33]*Gx2[31] + Gx1[34]*Gx2[41] + Gx1[35]*Gx2[51] + Gx1[36]*Gx2[61] + Gx1[37]*Gx2[71] + Gx1[38]*Gx2[81] + Gx1[39]*Gx2[91];
Gx3[32] = + Gx1[30]*Gx2[2] + Gx1[31]*Gx2[12] + Gx1[32]*Gx2[22] + Gx1[33]*Gx2[32] + Gx1[34]*Gx2[42] + Gx1[35]*Gx2[52] + Gx1[36]*Gx2[62] + Gx1[37]*Gx2[72] + Gx1[38]*Gx2[82] + Gx1[39]*Gx2[92];
Gx3[33] = + Gx1[30]*Gx2[3] + Gx1[31]*Gx2[13] + Gx1[32]*Gx2[23] + Gx1[33]*Gx2[33] + Gx1[34]*Gx2[43] + Gx1[35]*Gx2[53] + Gx1[36]*Gx2[63] + Gx1[37]*Gx2[73] + Gx1[38]*Gx2[83] + Gx1[39]*Gx2[93];
Gx3[34] = + Gx1[30]*Gx2[4] + Gx1[31]*Gx2[14] + Gx1[32]*Gx2[24] + Gx1[33]*Gx2[34] + Gx1[34]*Gx2[44] + Gx1[35]*Gx2[54] + Gx1[36]*Gx2[64] + Gx1[37]*Gx2[74] + Gx1[38]*Gx2[84] + Gx1[39]*Gx2[94];
Gx3[35] = + Gx1[30]*Gx2[5] + Gx1[31]*Gx2[15] + Gx1[32]*Gx2[25] + Gx1[33]*Gx2[35] + Gx1[34]*Gx2[45] + Gx1[35]*Gx2[55] + Gx1[36]*Gx2[65] + Gx1[37]*Gx2[75] + Gx1[38]*Gx2[85] + Gx1[39]*Gx2[95];
Gx3[36] = + Gx1[30]*Gx2[6] + Gx1[31]*Gx2[16] + Gx1[32]*Gx2[26] + Gx1[33]*Gx2[36] + Gx1[34]*Gx2[46] + Gx1[35]*Gx2[56] + Gx1[36]*Gx2[66] + Gx1[37]*Gx2[76] + Gx1[38]*Gx2[86] + Gx1[39]*Gx2[96];
Gx3[37] = + Gx1[30]*Gx2[7] + Gx1[31]*Gx2[17] + Gx1[32]*Gx2[27] + Gx1[33]*Gx2[37] + Gx1[34]*Gx2[47] + Gx1[35]*Gx2[57] + Gx1[36]*Gx2[67] + Gx1[37]*Gx2[77] + Gx1[38]*Gx2[87] + Gx1[39]*Gx2[97];
Gx3[38] = + Gx1[30]*Gx2[8] + Gx1[31]*Gx2[18] + Gx1[32]*Gx2[28] + Gx1[33]*Gx2[38] + Gx1[34]*Gx2[48] + Gx1[35]*Gx2[58] + Gx1[36]*Gx2[68] + Gx1[37]*Gx2[78] + Gx1[38]*Gx2[88] + Gx1[39]*Gx2[98];
Gx3[39] = + Gx1[30]*Gx2[9] + Gx1[31]*Gx2[19] + Gx1[32]*Gx2[29] + Gx1[33]*Gx2[39] + Gx1[34]*Gx2[49] + Gx1[35]*Gx2[59] + Gx1[36]*Gx2[69] + Gx1[37]*Gx2[79] + Gx1[38]*Gx2[89] + Gx1[39]*Gx2[99];
Gx3[40] = + Gx1[40]*Gx2[0] + Gx1[41]*Gx2[10] + Gx1[42]*Gx2[20] + Gx1[43]*Gx2[30] + Gx1[44]*Gx2[40] + Gx1[45]*Gx2[50] + Gx1[46]*Gx2[60] + Gx1[47]*Gx2[70] + Gx1[48]*Gx2[80] + Gx1[49]*Gx2[90];
Gx3[41] = + Gx1[40]*Gx2[1] + Gx1[41]*Gx2[11] + Gx1[42]*Gx2[21] + Gx1[43]*Gx2[31] + Gx1[44]*Gx2[41] + Gx1[45]*Gx2[51] + Gx1[46]*Gx2[61] + Gx1[47]*Gx2[71] + Gx1[48]*Gx2[81] + Gx1[49]*Gx2[91];
Gx3[42] = + Gx1[40]*Gx2[2] + Gx1[41]*Gx2[12] + Gx1[42]*Gx2[22] + Gx1[43]*Gx2[32] + Gx1[44]*Gx2[42] + Gx1[45]*Gx2[52] + Gx1[46]*Gx2[62] + Gx1[47]*Gx2[72] + Gx1[48]*Gx2[82] + Gx1[49]*Gx2[92];
Gx3[43] = + Gx1[40]*Gx2[3] + Gx1[41]*Gx2[13] + Gx1[42]*Gx2[23] + Gx1[43]*Gx2[33] + Gx1[44]*Gx2[43] + Gx1[45]*Gx2[53] + Gx1[46]*Gx2[63] + Gx1[47]*Gx2[73] + Gx1[48]*Gx2[83] + Gx1[49]*Gx2[93];
Gx3[44] = + Gx1[40]*Gx2[4] + Gx1[41]*Gx2[14] + Gx1[42]*Gx2[24] + Gx1[43]*Gx2[34] + Gx1[44]*Gx2[44] + Gx1[45]*Gx2[54] + Gx1[46]*Gx2[64] + Gx1[47]*Gx2[74] + Gx1[48]*Gx2[84] + Gx1[49]*Gx2[94];
Gx3[45] = + Gx1[40]*Gx2[5] + Gx1[41]*Gx2[15] + Gx1[42]*Gx2[25] + Gx1[43]*Gx2[35] + Gx1[44]*Gx2[45] + Gx1[45]*Gx2[55] + Gx1[46]*Gx2[65] + Gx1[47]*Gx2[75] + Gx1[48]*Gx2[85] + Gx1[49]*Gx2[95];
Gx3[46] = + Gx1[40]*Gx2[6] + Gx1[41]*Gx2[16] + Gx1[42]*Gx2[26] + Gx1[43]*Gx2[36] + Gx1[44]*Gx2[46] + Gx1[45]*Gx2[56] + Gx1[46]*Gx2[66] + Gx1[47]*Gx2[76] + Gx1[48]*Gx2[86] + Gx1[49]*Gx2[96];
Gx3[47] = + Gx1[40]*Gx2[7] + Gx1[41]*Gx2[17] + Gx1[42]*Gx2[27] + Gx1[43]*Gx2[37] + Gx1[44]*Gx2[47] + Gx1[45]*Gx2[57] + Gx1[46]*Gx2[67] + Gx1[47]*Gx2[77] + Gx1[48]*Gx2[87] + Gx1[49]*Gx2[97];
Gx3[48] = + Gx1[40]*Gx2[8] + Gx1[41]*Gx2[18] + Gx1[42]*Gx2[28] + Gx1[43]*Gx2[38] + Gx1[44]*Gx2[48] + Gx1[45]*Gx2[58] + Gx1[46]*Gx2[68] + Gx1[47]*Gx2[78] + Gx1[48]*Gx2[88] + Gx1[49]*Gx2[98];
Gx3[49] = + Gx1[40]*Gx2[9] + Gx1[41]*Gx2[19] + Gx1[42]*Gx2[29] + Gx1[43]*Gx2[39] + Gx1[44]*Gx2[49] + Gx1[45]*Gx2[59] + Gx1[46]*Gx2[69] + Gx1[47]*Gx2[79] + Gx1[48]*Gx2[89] + Gx1[49]*Gx2[99];
Gx3[50] = + Gx1[50]*Gx2[0] + Gx1[51]*Gx2[10] + Gx1[52]*Gx2[20] + Gx1[53]*Gx2[30] + Gx1[54]*Gx2[40] + Gx1[55]*Gx2[50] + Gx1[56]*Gx2[60] + Gx1[57]*Gx2[70] + Gx1[58]*Gx2[80] + Gx1[59]*Gx2[90];
Gx3[51] = + Gx1[50]*Gx2[1] + Gx1[51]*Gx2[11] + Gx1[52]*Gx2[21] + Gx1[53]*Gx2[31] + Gx1[54]*Gx2[41] + Gx1[55]*Gx2[51] + Gx1[56]*Gx2[61] + Gx1[57]*Gx2[71] + Gx1[58]*Gx2[81] + Gx1[59]*Gx2[91];
Gx3[52] = + Gx1[50]*Gx2[2] + Gx1[51]*Gx2[12] + Gx1[52]*Gx2[22] + Gx1[53]*Gx2[32] + Gx1[54]*Gx2[42] + Gx1[55]*Gx2[52] + Gx1[56]*Gx2[62] + Gx1[57]*Gx2[72] + Gx1[58]*Gx2[82] + Gx1[59]*Gx2[92];
Gx3[53] = + Gx1[50]*Gx2[3] + Gx1[51]*Gx2[13] + Gx1[52]*Gx2[23] + Gx1[53]*Gx2[33] + Gx1[54]*Gx2[43] + Gx1[55]*Gx2[53] + Gx1[56]*Gx2[63] + Gx1[57]*Gx2[73] + Gx1[58]*Gx2[83] + Gx1[59]*Gx2[93];
Gx3[54] = + Gx1[50]*Gx2[4] + Gx1[51]*Gx2[14] + Gx1[52]*Gx2[24] + Gx1[53]*Gx2[34] + Gx1[54]*Gx2[44] + Gx1[55]*Gx2[54] + Gx1[56]*Gx2[64] + Gx1[57]*Gx2[74] + Gx1[58]*Gx2[84] + Gx1[59]*Gx2[94];
Gx3[55] = + Gx1[50]*Gx2[5] + Gx1[51]*Gx2[15] + Gx1[52]*Gx2[25] + Gx1[53]*Gx2[35] + Gx1[54]*Gx2[45] + Gx1[55]*Gx2[55] + Gx1[56]*Gx2[65] + Gx1[57]*Gx2[75] + Gx1[58]*Gx2[85] + Gx1[59]*Gx2[95];
Gx3[56] = + Gx1[50]*Gx2[6] + Gx1[51]*Gx2[16] + Gx1[52]*Gx2[26] + Gx1[53]*Gx2[36] + Gx1[54]*Gx2[46] + Gx1[55]*Gx2[56] + Gx1[56]*Gx2[66] + Gx1[57]*Gx2[76] + Gx1[58]*Gx2[86] + Gx1[59]*Gx2[96];
Gx3[57] = + Gx1[50]*Gx2[7] + Gx1[51]*Gx2[17] + Gx1[52]*Gx2[27] + Gx1[53]*Gx2[37] + Gx1[54]*Gx2[47] + Gx1[55]*Gx2[57] + Gx1[56]*Gx2[67] + Gx1[57]*Gx2[77] + Gx1[58]*Gx2[87] + Gx1[59]*Gx2[97];
Gx3[58] = + Gx1[50]*Gx2[8] + Gx1[51]*Gx2[18] + Gx1[52]*Gx2[28] + Gx1[53]*Gx2[38] + Gx1[54]*Gx2[48] + Gx1[55]*Gx2[58] + Gx1[56]*Gx2[68] + Gx1[57]*Gx2[78] + Gx1[58]*Gx2[88] + Gx1[59]*Gx2[98];
Gx3[59] = + Gx1[50]*Gx2[9] + Gx1[51]*Gx2[19] + Gx1[52]*Gx2[29] + Gx1[53]*Gx2[39] + Gx1[54]*Gx2[49] + Gx1[55]*Gx2[59] + Gx1[56]*Gx2[69] + Gx1[57]*Gx2[79] + Gx1[58]*Gx2[89] + Gx1[59]*Gx2[99];
Gx3[60] = + Gx1[60]*Gx2[0] + Gx1[61]*Gx2[10] + Gx1[62]*Gx2[20] + Gx1[63]*Gx2[30] + Gx1[64]*Gx2[40] + Gx1[65]*Gx2[50] + Gx1[66]*Gx2[60] + Gx1[67]*Gx2[70] + Gx1[68]*Gx2[80] + Gx1[69]*Gx2[90];
Gx3[61] = + Gx1[60]*Gx2[1] + Gx1[61]*Gx2[11] + Gx1[62]*Gx2[21] + Gx1[63]*Gx2[31] + Gx1[64]*Gx2[41] + Gx1[65]*Gx2[51] + Gx1[66]*Gx2[61] + Gx1[67]*Gx2[71] + Gx1[68]*Gx2[81] + Gx1[69]*Gx2[91];
Gx3[62] = + Gx1[60]*Gx2[2] + Gx1[61]*Gx2[12] + Gx1[62]*Gx2[22] + Gx1[63]*Gx2[32] + Gx1[64]*Gx2[42] + Gx1[65]*Gx2[52] + Gx1[66]*Gx2[62] + Gx1[67]*Gx2[72] + Gx1[68]*Gx2[82] + Gx1[69]*Gx2[92];
Gx3[63] = + Gx1[60]*Gx2[3] + Gx1[61]*Gx2[13] + Gx1[62]*Gx2[23] + Gx1[63]*Gx2[33] + Gx1[64]*Gx2[43] + Gx1[65]*Gx2[53] + Gx1[66]*Gx2[63] + Gx1[67]*Gx2[73] + Gx1[68]*Gx2[83] + Gx1[69]*Gx2[93];
Gx3[64] = + Gx1[60]*Gx2[4] + Gx1[61]*Gx2[14] + Gx1[62]*Gx2[24] + Gx1[63]*Gx2[34] + Gx1[64]*Gx2[44] + Gx1[65]*Gx2[54] + Gx1[66]*Gx2[64] + Gx1[67]*Gx2[74] + Gx1[68]*Gx2[84] + Gx1[69]*Gx2[94];
Gx3[65] = + Gx1[60]*Gx2[5] + Gx1[61]*Gx2[15] + Gx1[62]*Gx2[25] + Gx1[63]*Gx2[35] + Gx1[64]*Gx2[45] + Gx1[65]*Gx2[55] + Gx1[66]*Gx2[65] + Gx1[67]*Gx2[75] + Gx1[68]*Gx2[85] + Gx1[69]*Gx2[95];
Gx3[66] = + Gx1[60]*Gx2[6] + Gx1[61]*Gx2[16] + Gx1[62]*Gx2[26] + Gx1[63]*Gx2[36] + Gx1[64]*Gx2[46] + Gx1[65]*Gx2[56] + Gx1[66]*Gx2[66] + Gx1[67]*Gx2[76] + Gx1[68]*Gx2[86] + Gx1[69]*Gx2[96];
Gx3[67] = + Gx1[60]*Gx2[7] + Gx1[61]*Gx2[17] + Gx1[62]*Gx2[27] + Gx1[63]*Gx2[37] + Gx1[64]*Gx2[47] + Gx1[65]*Gx2[57] + Gx1[66]*Gx2[67] + Gx1[67]*Gx2[77] + Gx1[68]*Gx2[87] + Gx1[69]*Gx2[97];
Gx3[68] = + Gx1[60]*Gx2[8] + Gx1[61]*Gx2[18] + Gx1[62]*Gx2[28] + Gx1[63]*Gx2[38] + Gx1[64]*Gx2[48] + Gx1[65]*Gx2[58] + Gx1[66]*Gx2[68] + Gx1[67]*Gx2[78] + Gx1[68]*Gx2[88] + Gx1[69]*Gx2[98];
Gx3[69] = + Gx1[60]*Gx2[9] + Gx1[61]*Gx2[19] + Gx1[62]*Gx2[29] + Gx1[63]*Gx2[39] + Gx1[64]*Gx2[49] + Gx1[65]*Gx2[59] + Gx1[66]*Gx2[69] + Gx1[67]*Gx2[79] + Gx1[68]*Gx2[89] + Gx1[69]*Gx2[99];
Gx3[70] = + Gx1[70]*Gx2[0] + Gx1[71]*Gx2[10] + Gx1[72]*Gx2[20] + Gx1[73]*Gx2[30] + Gx1[74]*Gx2[40] + Gx1[75]*Gx2[50] + Gx1[76]*Gx2[60] + Gx1[77]*Gx2[70] + Gx1[78]*Gx2[80] + Gx1[79]*Gx2[90];
Gx3[71] = + Gx1[70]*Gx2[1] + Gx1[71]*Gx2[11] + Gx1[72]*Gx2[21] + Gx1[73]*Gx2[31] + Gx1[74]*Gx2[41] + Gx1[75]*Gx2[51] + Gx1[76]*Gx2[61] + Gx1[77]*Gx2[71] + Gx1[78]*Gx2[81] + Gx1[79]*Gx2[91];
Gx3[72] = + Gx1[70]*Gx2[2] + Gx1[71]*Gx2[12] + Gx1[72]*Gx2[22] + Gx1[73]*Gx2[32] + Gx1[74]*Gx2[42] + Gx1[75]*Gx2[52] + Gx1[76]*Gx2[62] + Gx1[77]*Gx2[72] + Gx1[78]*Gx2[82] + Gx1[79]*Gx2[92];
Gx3[73] = + Gx1[70]*Gx2[3] + Gx1[71]*Gx2[13] + Gx1[72]*Gx2[23] + Gx1[73]*Gx2[33] + Gx1[74]*Gx2[43] + Gx1[75]*Gx2[53] + Gx1[76]*Gx2[63] + Gx1[77]*Gx2[73] + Gx1[78]*Gx2[83] + Gx1[79]*Gx2[93];
Gx3[74] = + Gx1[70]*Gx2[4] + Gx1[71]*Gx2[14] + Gx1[72]*Gx2[24] + Gx1[73]*Gx2[34] + Gx1[74]*Gx2[44] + Gx1[75]*Gx2[54] + Gx1[76]*Gx2[64] + Gx1[77]*Gx2[74] + Gx1[78]*Gx2[84] + Gx1[79]*Gx2[94];
Gx3[75] = + Gx1[70]*Gx2[5] + Gx1[71]*Gx2[15] + Gx1[72]*Gx2[25] + Gx1[73]*Gx2[35] + Gx1[74]*Gx2[45] + Gx1[75]*Gx2[55] + Gx1[76]*Gx2[65] + Gx1[77]*Gx2[75] + Gx1[78]*Gx2[85] + Gx1[79]*Gx2[95];
Gx3[76] = + Gx1[70]*Gx2[6] + Gx1[71]*Gx2[16] + Gx1[72]*Gx2[26] + Gx1[73]*Gx2[36] + Gx1[74]*Gx2[46] + Gx1[75]*Gx2[56] + Gx1[76]*Gx2[66] + Gx1[77]*Gx2[76] + Gx1[78]*Gx2[86] + Gx1[79]*Gx2[96];
Gx3[77] = + Gx1[70]*Gx2[7] + Gx1[71]*Gx2[17] + Gx1[72]*Gx2[27] + Gx1[73]*Gx2[37] + Gx1[74]*Gx2[47] + Gx1[75]*Gx2[57] + Gx1[76]*Gx2[67] + Gx1[77]*Gx2[77] + Gx1[78]*Gx2[87] + Gx1[79]*Gx2[97];
Gx3[78] = + Gx1[70]*Gx2[8] + Gx1[71]*Gx2[18] + Gx1[72]*Gx2[28] + Gx1[73]*Gx2[38] + Gx1[74]*Gx2[48] + Gx1[75]*Gx2[58] + Gx1[76]*Gx2[68] + Gx1[77]*Gx2[78] + Gx1[78]*Gx2[88] + Gx1[79]*Gx2[98];
Gx3[79] = + Gx1[70]*Gx2[9] + Gx1[71]*Gx2[19] + Gx1[72]*Gx2[29] + Gx1[73]*Gx2[39] + Gx1[74]*Gx2[49] + Gx1[75]*Gx2[59] + Gx1[76]*Gx2[69] + Gx1[77]*Gx2[79] + Gx1[78]*Gx2[89] + Gx1[79]*Gx2[99];
Gx3[80] = + Gx1[80]*Gx2[0] + Gx1[81]*Gx2[10] + Gx1[82]*Gx2[20] + Gx1[83]*Gx2[30] + Gx1[84]*Gx2[40] + Gx1[85]*Gx2[50] + Gx1[86]*Gx2[60] + Gx1[87]*Gx2[70] + Gx1[88]*Gx2[80] + Gx1[89]*Gx2[90];
Gx3[81] = + Gx1[80]*Gx2[1] + Gx1[81]*Gx2[11] + Gx1[82]*Gx2[21] + Gx1[83]*Gx2[31] + Gx1[84]*Gx2[41] + Gx1[85]*Gx2[51] + Gx1[86]*Gx2[61] + Gx1[87]*Gx2[71] + Gx1[88]*Gx2[81] + Gx1[89]*Gx2[91];
Gx3[82] = + Gx1[80]*Gx2[2] + Gx1[81]*Gx2[12] + Gx1[82]*Gx2[22] + Gx1[83]*Gx2[32] + Gx1[84]*Gx2[42] + Gx1[85]*Gx2[52] + Gx1[86]*Gx2[62] + Gx1[87]*Gx2[72] + Gx1[88]*Gx2[82] + Gx1[89]*Gx2[92];
Gx3[83] = + Gx1[80]*Gx2[3] + Gx1[81]*Gx2[13] + Gx1[82]*Gx2[23] + Gx1[83]*Gx2[33] + Gx1[84]*Gx2[43] + Gx1[85]*Gx2[53] + Gx1[86]*Gx2[63] + Gx1[87]*Gx2[73] + Gx1[88]*Gx2[83] + Gx1[89]*Gx2[93];
Gx3[84] = + Gx1[80]*Gx2[4] + Gx1[81]*Gx2[14] + Gx1[82]*Gx2[24] + Gx1[83]*Gx2[34] + Gx1[84]*Gx2[44] + Gx1[85]*Gx2[54] + Gx1[86]*Gx2[64] + Gx1[87]*Gx2[74] + Gx1[88]*Gx2[84] + Gx1[89]*Gx2[94];
Gx3[85] = + Gx1[80]*Gx2[5] + Gx1[81]*Gx2[15] + Gx1[82]*Gx2[25] + Gx1[83]*Gx2[35] + Gx1[84]*Gx2[45] + Gx1[85]*Gx2[55] + Gx1[86]*Gx2[65] + Gx1[87]*Gx2[75] + Gx1[88]*Gx2[85] + Gx1[89]*Gx2[95];
Gx3[86] = + Gx1[80]*Gx2[6] + Gx1[81]*Gx2[16] + Gx1[82]*Gx2[26] + Gx1[83]*Gx2[36] + Gx1[84]*Gx2[46] + Gx1[85]*Gx2[56] + Gx1[86]*Gx2[66] + Gx1[87]*Gx2[76] + Gx1[88]*Gx2[86] + Gx1[89]*Gx2[96];
Gx3[87] = + Gx1[80]*Gx2[7] + Gx1[81]*Gx2[17] + Gx1[82]*Gx2[27] + Gx1[83]*Gx2[37] + Gx1[84]*Gx2[47] + Gx1[85]*Gx2[57] + Gx1[86]*Gx2[67] + Gx1[87]*Gx2[77] + Gx1[88]*Gx2[87] + Gx1[89]*Gx2[97];
Gx3[88] = + Gx1[80]*Gx2[8] + Gx1[81]*Gx2[18] + Gx1[82]*Gx2[28] + Gx1[83]*Gx2[38] + Gx1[84]*Gx2[48] + Gx1[85]*Gx2[58] + Gx1[86]*Gx2[68] + Gx1[87]*Gx2[78] + Gx1[88]*Gx2[88] + Gx1[89]*Gx2[98];
Gx3[89] = + Gx1[80]*Gx2[9] + Gx1[81]*Gx2[19] + Gx1[82]*Gx2[29] + Gx1[83]*Gx2[39] + Gx1[84]*Gx2[49] + Gx1[85]*Gx2[59] + Gx1[86]*Gx2[69] + Gx1[87]*Gx2[79] + Gx1[88]*Gx2[89] + Gx1[89]*Gx2[99];
Gx3[90] = + Gx1[90]*Gx2[0] + Gx1[91]*Gx2[10] + Gx1[92]*Gx2[20] + Gx1[93]*Gx2[30] + Gx1[94]*Gx2[40] + Gx1[95]*Gx2[50] + Gx1[96]*Gx2[60] + Gx1[97]*Gx2[70] + Gx1[98]*Gx2[80] + Gx1[99]*Gx2[90];
Gx3[91] = + Gx1[90]*Gx2[1] + Gx1[91]*Gx2[11] + Gx1[92]*Gx2[21] + Gx1[93]*Gx2[31] + Gx1[94]*Gx2[41] + Gx1[95]*Gx2[51] + Gx1[96]*Gx2[61] + Gx1[97]*Gx2[71] + Gx1[98]*Gx2[81] + Gx1[99]*Gx2[91];
Gx3[92] = + Gx1[90]*Gx2[2] + Gx1[91]*Gx2[12] + Gx1[92]*Gx2[22] + Gx1[93]*Gx2[32] + Gx1[94]*Gx2[42] + Gx1[95]*Gx2[52] + Gx1[96]*Gx2[62] + Gx1[97]*Gx2[72] + Gx1[98]*Gx2[82] + Gx1[99]*Gx2[92];
Gx3[93] = + Gx1[90]*Gx2[3] + Gx1[91]*Gx2[13] + Gx1[92]*Gx2[23] + Gx1[93]*Gx2[33] + Gx1[94]*Gx2[43] + Gx1[95]*Gx2[53] + Gx1[96]*Gx2[63] + Gx1[97]*Gx2[73] + Gx1[98]*Gx2[83] + Gx1[99]*Gx2[93];
Gx3[94] = + Gx1[90]*Gx2[4] + Gx1[91]*Gx2[14] + Gx1[92]*Gx2[24] + Gx1[93]*Gx2[34] + Gx1[94]*Gx2[44] + Gx1[95]*Gx2[54] + Gx1[96]*Gx2[64] + Gx1[97]*Gx2[74] + Gx1[98]*Gx2[84] + Gx1[99]*Gx2[94];
Gx3[95] = + Gx1[90]*Gx2[5] + Gx1[91]*Gx2[15] + Gx1[92]*Gx2[25] + Gx1[93]*Gx2[35] + Gx1[94]*Gx2[45] + Gx1[95]*Gx2[55] + Gx1[96]*Gx2[65] + Gx1[97]*Gx2[75] + Gx1[98]*Gx2[85] + Gx1[99]*Gx2[95];
Gx3[96] = + Gx1[90]*Gx2[6] + Gx1[91]*Gx2[16] + Gx1[92]*Gx2[26] + Gx1[93]*Gx2[36] + Gx1[94]*Gx2[46] + Gx1[95]*Gx2[56] + Gx1[96]*Gx2[66] + Gx1[97]*Gx2[76] + Gx1[98]*Gx2[86] + Gx1[99]*Gx2[96];
Gx3[97] = + Gx1[90]*Gx2[7] + Gx1[91]*Gx2[17] + Gx1[92]*Gx2[27] + Gx1[93]*Gx2[37] + Gx1[94]*Gx2[47] + Gx1[95]*Gx2[57] + Gx1[96]*Gx2[67] + Gx1[97]*Gx2[77] + Gx1[98]*Gx2[87] + Gx1[99]*Gx2[97];
Gx3[98] = + Gx1[90]*Gx2[8] + Gx1[91]*Gx2[18] + Gx1[92]*Gx2[28] + Gx1[93]*Gx2[38] + Gx1[94]*Gx2[48] + Gx1[95]*Gx2[58] + Gx1[96]*Gx2[68] + Gx1[97]*Gx2[78] + Gx1[98]*Gx2[88] + Gx1[99]*Gx2[98];
Gx3[99] = + Gx1[90]*Gx2[9] + Gx1[91]*Gx2[19] + Gx1[92]*Gx2[29] + Gx1[93]*Gx2[39] + Gx1[94]*Gx2[49] + Gx1[95]*Gx2[59] + Gx1[96]*Gx2[69] + Gx1[97]*Gx2[79] + Gx1[98]*Gx2[89] + Gx1[99]*Gx2[99];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[4] + Gx1[12]*Gu1[8] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[24] + Gx1[17]*Gu1[28] + Gx1[18]*Gu1[32] + Gx1[19]*Gu1[36];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[5] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[21] + Gx1[16]*Gu1[25] + Gx1[17]*Gu1[29] + Gx1[18]*Gu1[33] + Gx1[19]*Gu1[37];
Gu2[6] = + Gx1[10]*Gu1[2] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[18] + Gx1[15]*Gu1[22] + Gx1[16]*Gu1[26] + Gx1[17]*Gu1[30] + Gx1[18]*Gu1[34] + Gx1[19]*Gu1[38];
Gu2[7] = + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[15] + Gx1[14]*Gu1[19] + Gx1[15]*Gu1[23] + Gx1[16]*Gu1[27] + Gx1[17]*Gu1[31] + Gx1[18]*Gu1[35] + Gx1[19]*Gu1[39];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[4] + Gx1[22]*Gu1[8] + Gx1[23]*Gu1[12] + Gx1[24]*Gu1[16] + Gx1[25]*Gu1[20] + Gx1[26]*Gu1[24] + Gx1[27]*Gu1[28] + Gx1[28]*Gu1[32] + Gx1[29]*Gu1[36];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[5] + Gx1[22]*Gu1[9] + Gx1[23]*Gu1[13] + Gx1[24]*Gu1[17] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[25] + Gx1[27]*Gu1[29] + Gx1[28]*Gu1[33] + Gx1[29]*Gu1[37];
Gu2[10] = + Gx1[20]*Gu1[2] + Gx1[21]*Gu1[6] + Gx1[22]*Gu1[10] + Gx1[23]*Gu1[14] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[26] + Gx1[27]*Gu1[30] + Gx1[28]*Gu1[34] + Gx1[29]*Gu1[38];
Gu2[11] = + Gx1[20]*Gu1[3] + Gx1[21]*Gu1[7] + Gx1[22]*Gu1[11] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[27] + Gx1[27]*Gu1[31] + Gx1[28]*Gu1[35] + Gx1[29]*Gu1[39];
Gu2[12] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[4] + Gx1[32]*Gu1[8] + Gx1[33]*Gu1[12] + Gx1[34]*Gu1[16] + Gx1[35]*Gu1[20] + Gx1[36]*Gu1[24] + Gx1[37]*Gu1[28] + Gx1[38]*Gu1[32] + Gx1[39]*Gu1[36];
Gu2[13] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[5] + Gx1[32]*Gu1[9] + Gx1[33]*Gu1[13] + Gx1[34]*Gu1[17] + Gx1[35]*Gu1[21] + Gx1[36]*Gu1[25] + Gx1[37]*Gu1[29] + Gx1[38]*Gu1[33] + Gx1[39]*Gu1[37];
Gu2[14] = + Gx1[30]*Gu1[2] + Gx1[31]*Gu1[6] + Gx1[32]*Gu1[10] + Gx1[33]*Gu1[14] + Gx1[34]*Gu1[18] + Gx1[35]*Gu1[22] + Gx1[36]*Gu1[26] + Gx1[37]*Gu1[30] + Gx1[38]*Gu1[34] + Gx1[39]*Gu1[38];
Gu2[15] = + Gx1[30]*Gu1[3] + Gx1[31]*Gu1[7] + Gx1[32]*Gu1[11] + Gx1[33]*Gu1[15] + Gx1[34]*Gu1[19] + Gx1[35]*Gu1[23] + Gx1[36]*Gu1[27] + Gx1[37]*Gu1[31] + Gx1[38]*Gu1[35] + Gx1[39]*Gu1[39];
Gu2[16] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[4] + Gx1[42]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[45]*Gu1[20] + Gx1[46]*Gu1[24] + Gx1[47]*Gu1[28] + Gx1[48]*Gu1[32] + Gx1[49]*Gu1[36];
Gu2[17] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[5] + Gx1[42]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[45]*Gu1[21] + Gx1[46]*Gu1[25] + Gx1[47]*Gu1[29] + Gx1[48]*Gu1[33] + Gx1[49]*Gu1[37];
Gu2[18] = + Gx1[40]*Gu1[2] + Gx1[41]*Gu1[6] + Gx1[42]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[45]*Gu1[22] + Gx1[46]*Gu1[26] + Gx1[47]*Gu1[30] + Gx1[48]*Gu1[34] + Gx1[49]*Gu1[38];
Gu2[19] = + Gx1[40]*Gu1[3] + Gx1[41]*Gu1[7] + Gx1[42]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[45]*Gu1[23] + Gx1[46]*Gu1[27] + Gx1[47]*Gu1[31] + Gx1[48]*Gu1[35] + Gx1[49]*Gu1[39];
Gu2[20] = + Gx1[50]*Gu1[0] + Gx1[51]*Gu1[4] + Gx1[52]*Gu1[8] + Gx1[53]*Gu1[12] + Gx1[54]*Gu1[16] + Gx1[55]*Gu1[20] + Gx1[56]*Gu1[24] + Gx1[57]*Gu1[28] + Gx1[58]*Gu1[32] + Gx1[59]*Gu1[36];
Gu2[21] = + Gx1[50]*Gu1[1] + Gx1[51]*Gu1[5] + Gx1[52]*Gu1[9] + Gx1[53]*Gu1[13] + Gx1[54]*Gu1[17] + Gx1[55]*Gu1[21] + Gx1[56]*Gu1[25] + Gx1[57]*Gu1[29] + Gx1[58]*Gu1[33] + Gx1[59]*Gu1[37];
Gu2[22] = + Gx1[50]*Gu1[2] + Gx1[51]*Gu1[6] + Gx1[52]*Gu1[10] + Gx1[53]*Gu1[14] + Gx1[54]*Gu1[18] + Gx1[55]*Gu1[22] + Gx1[56]*Gu1[26] + Gx1[57]*Gu1[30] + Gx1[58]*Gu1[34] + Gx1[59]*Gu1[38];
Gu2[23] = + Gx1[50]*Gu1[3] + Gx1[51]*Gu1[7] + Gx1[52]*Gu1[11] + Gx1[53]*Gu1[15] + Gx1[54]*Gu1[19] + Gx1[55]*Gu1[23] + Gx1[56]*Gu1[27] + Gx1[57]*Gu1[31] + Gx1[58]*Gu1[35] + Gx1[59]*Gu1[39];
Gu2[24] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36];
Gu2[25] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37];
Gu2[26] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38];
Gu2[27] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39];
Gu2[28] = + Gx1[70]*Gu1[0] + Gx1[71]*Gu1[4] + Gx1[72]*Gu1[8] + Gx1[73]*Gu1[12] + Gx1[74]*Gu1[16] + Gx1[75]*Gu1[20] + Gx1[76]*Gu1[24] + Gx1[77]*Gu1[28] + Gx1[78]*Gu1[32] + Gx1[79]*Gu1[36];
Gu2[29] = + Gx1[70]*Gu1[1] + Gx1[71]*Gu1[5] + Gx1[72]*Gu1[9] + Gx1[73]*Gu1[13] + Gx1[74]*Gu1[17] + Gx1[75]*Gu1[21] + Gx1[76]*Gu1[25] + Gx1[77]*Gu1[29] + Gx1[78]*Gu1[33] + Gx1[79]*Gu1[37];
Gu2[30] = + Gx1[70]*Gu1[2] + Gx1[71]*Gu1[6] + Gx1[72]*Gu1[10] + Gx1[73]*Gu1[14] + Gx1[74]*Gu1[18] + Gx1[75]*Gu1[22] + Gx1[76]*Gu1[26] + Gx1[77]*Gu1[30] + Gx1[78]*Gu1[34] + Gx1[79]*Gu1[38];
Gu2[31] = + Gx1[70]*Gu1[3] + Gx1[71]*Gu1[7] + Gx1[72]*Gu1[11] + Gx1[73]*Gu1[15] + Gx1[74]*Gu1[19] + Gx1[75]*Gu1[23] + Gx1[76]*Gu1[27] + Gx1[77]*Gu1[31] + Gx1[78]*Gu1[35] + Gx1[79]*Gu1[39];
Gu2[32] = + Gx1[80]*Gu1[0] + Gx1[81]*Gu1[4] + Gx1[82]*Gu1[8] + Gx1[83]*Gu1[12] + Gx1[84]*Gu1[16] + Gx1[85]*Gu1[20] + Gx1[86]*Gu1[24] + Gx1[87]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[89]*Gu1[36];
Gu2[33] = + Gx1[80]*Gu1[1] + Gx1[81]*Gu1[5] + Gx1[82]*Gu1[9] + Gx1[83]*Gu1[13] + Gx1[84]*Gu1[17] + Gx1[85]*Gu1[21] + Gx1[86]*Gu1[25] + Gx1[87]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[89]*Gu1[37];
Gu2[34] = + Gx1[80]*Gu1[2] + Gx1[81]*Gu1[6] + Gx1[82]*Gu1[10] + Gx1[83]*Gu1[14] + Gx1[84]*Gu1[18] + Gx1[85]*Gu1[22] + Gx1[86]*Gu1[26] + Gx1[87]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[89]*Gu1[38];
Gu2[35] = + Gx1[80]*Gu1[3] + Gx1[81]*Gu1[7] + Gx1[82]*Gu1[11] + Gx1[83]*Gu1[15] + Gx1[84]*Gu1[19] + Gx1[85]*Gu1[23] + Gx1[86]*Gu1[27] + Gx1[87]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[89]*Gu1[39];
Gu2[36] = + Gx1[90]*Gu1[0] + Gx1[91]*Gu1[4] + Gx1[92]*Gu1[8] + Gx1[93]*Gu1[12] + Gx1[94]*Gu1[16] + Gx1[95]*Gu1[20] + Gx1[96]*Gu1[24] + Gx1[97]*Gu1[28] + Gx1[98]*Gu1[32] + Gx1[99]*Gu1[36];
Gu2[37] = + Gx1[90]*Gu1[1] + Gx1[91]*Gu1[5] + Gx1[92]*Gu1[9] + Gx1[93]*Gu1[13] + Gx1[94]*Gu1[17] + Gx1[95]*Gu1[21] + Gx1[96]*Gu1[25] + Gx1[97]*Gu1[29] + Gx1[98]*Gu1[33] + Gx1[99]*Gu1[37];
Gu2[38] = + Gx1[90]*Gu1[2] + Gx1[91]*Gu1[6] + Gx1[92]*Gu1[10] + Gx1[93]*Gu1[14] + Gx1[94]*Gu1[18] + Gx1[95]*Gu1[22] + Gx1[96]*Gu1[26] + Gx1[97]*Gu1[30] + Gx1[98]*Gu1[34] + Gx1[99]*Gu1[38];
Gu2[39] = + Gx1[90]*Gu1[3] + Gx1[91]*Gu1[7] + Gx1[92]*Gu1[11] + Gx1[93]*Gu1[15] + Gx1[94]*Gu1[19] + Gx1[95]*Gu1[23] + Gx1[96]*Gu1[27] + Gx1[97]*Gu1[31] + Gx1[98]*Gu1[35] + Gx1[99]*Gu1[39];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 324] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + R11[0];
acadoWorkspace.H[iRow * 324 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + R11[1];
acadoWorkspace.H[iRow * 324 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + R11[2];
acadoWorkspace.H[iRow * 324 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + R11[3];
acadoWorkspace.H[iRow * 324 + 80] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + R11[4];
acadoWorkspace.H[iRow * 324 + 81] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + R11[5];
acadoWorkspace.H[iRow * 324 + 82] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + R11[6];
acadoWorkspace.H[iRow * 324 + 83] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + R11[7];
acadoWorkspace.H[iRow * 324 + 160] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + R11[8];
acadoWorkspace.H[iRow * 324 + 161] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + R11[9];
acadoWorkspace.H[iRow * 324 + 162] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + R11[10];
acadoWorkspace.H[iRow * 324 + 163] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + R11[11];
acadoWorkspace.H[iRow * 324 + 240] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + R11[12];
acadoWorkspace.H[iRow * 324 + 241] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + R11[13];
acadoWorkspace.H[iRow * 324 + 242] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + R11[14];
acadoWorkspace.H[iRow * 324 + 243] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + R11[15];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[10]*Gu1[4] + Gx1[20]*Gu1[8] + Gx1[30]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[50]*Gu1[20] + Gx1[60]*Gu1[24] + Gx1[70]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[90]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[10]*Gu1[5] + Gx1[20]*Gu1[9] + Gx1[30]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[50]*Gu1[21] + Gx1[60]*Gu1[25] + Gx1[70]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[90]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[10]*Gu1[6] + Gx1[20]*Gu1[10] + Gx1[30]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[50]*Gu1[22] + Gx1[60]*Gu1[26] + Gx1[70]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[90]*Gu1[38];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[10]*Gu1[7] + Gx1[20]*Gu1[11] + Gx1[30]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[50]*Gu1[23] + Gx1[60]*Gu1[27] + Gx1[70]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[90]*Gu1[39];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[11]*Gu1[4] + Gx1[21]*Gu1[8] + Gx1[31]*Gu1[12] + Gx1[41]*Gu1[16] + Gx1[51]*Gu1[20] + Gx1[61]*Gu1[24] + Gx1[71]*Gu1[28] + Gx1[81]*Gu1[32] + Gx1[91]*Gu1[36];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[11]*Gu1[5] + Gx1[21]*Gu1[9] + Gx1[31]*Gu1[13] + Gx1[41]*Gu1[17] + Gx1[51]*Gu1[21] + Gx1[61]*Gu1[25] + Gx1[71]*Gu1[29] + Gx1[81]*Gu1[33] + Gx1[91]*Gu1[37];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[11]*Gu1[6] + Gx1[21]*Gu1[10] + Gx1[31]*Gu1[14] + Gx1[41]*Gu1[18] + Gx1[51]*Gu1[22] + Gx1[61]*Gu1[26] + Gx1[71]*Gu1[30] + Gx1[81]*Gu1[34] + Gx1[91]*Gu1[38];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[11]*Gu1[7] + Gx1[21]*Gu1[11] + Gx1[31]*Gu1[15] + Gx1[41]*Gu1[19] + Gx1[51]*Gu1[23] + Gx1[61]*Gu1[27] + Gx1[71]*Gu1[31] + Gx1[81]*Gu1[35] + Gx1[91]*Gu1[39];
Gu2[8] = + Gx1[2]*Gu1[0] + Gx1[12]*Gu1[4] + Gx1[22]*Gu1[8] + Gx1[32]*Gu1[12] + Gx1[42]*Gu1[16] + Gx1[52]*Gu1[20] + Gx1[62]*Gu1[24] + Gx1[72]*Gu1[28] + Gx1[82]*Gu1[32] + Gx1[92]*Gu1[36];
Gu2[9] = + Gx1[2]*Gu1[1] + Gx1[12]*Gu1[5] + Gx1[22]*Gu1[9] + Gx1[32]*Gu1[13] + Gx1[42]*Gu1[17] + Gx1[52]*Gu1[21] + Gx1[62]*Gu1[25] + Gx1[72]*Gu1[29] + Gx1[82]*Gu1[33] + Gx1[92]*Gu1[37];
Gu2[10] = + Gx1[2]*Gu1[2] + Gx1[12]*Gu1[6] + Gx1[22]*Gu1[10] + Gx1[32]*Gu1[14] + Gx1[42]*Gu1[18] + Gx1[52]*Gu1[22] + Gx1[62]*Gu1[26] + Gx1[72]*Gu1[30] + Gx1[82]*Gu1[34] + Gx1[92]*Gu1[38];
Gu2[11] = + Gx1[2]*Gu1[3] + Gx1[12]*Gu1[7] + Gx1[22]*Gu1[11] + Gx1[32]*Gu1[15] + Gx1[42]*Gu1[19] + Gx1[52]*Gu1[23] + Gx1[62]*Gu1[27] + Gx1[72]*Gu1[31] + Gx1[82]*Gu1[35] + Gx1[92]*Gu1[39];
Gu2[12] = + Gx1[3]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[23]*Gu1[8] + Gx1[33]*Gu1[12] + Gx1[43]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[63]*Gu1[24] + Gx1[73]*Gu1[28] + Gx1[83]*Gu1[32] + Gx1[93]*Gu1[36];
Gu2[13] = + Gx1[3]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[23]*Gu1[9] + Gx1[33]*Gu1[13] + Gx1[43]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[63]*Gu1[25] + Gx1[73]*Gu1[29] + Gx1[83]*Gu1[33] + Gx1[93]*Gu1[37];
Gu2[14] = + Gx1[3]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[23]*Gu1[10] + Gx1[33]*Gu1[14] + Gx1[43]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[63]*Gu1[26] + Gx1[73]*Gu1[30] + Gx1[83]*Gu1[34] + Gx1[93]*Gu1[38];
Gu2[15] = + Gx1[3]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[23]*Gu1[11] + Gx1[33]*Gu1[15] + Gx1[43]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[63]*Gu1[27] + Gx1[73]*Gu1[31] + Gx1[83]*Gu1[35] + Gx1[93]*Gu1[39];
Gu2[16] = + Gx1[4]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[24]*Gu1[8] + Gx1[34]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[54]*Gu1[20] + Gx1[64]*Gu1[24] + Gx1[74]*Gu1[28] + Gx1[84]*Gu1[32] + Gx1[94]*Gu1[36];
Gu2[17] = + Gx1[4]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[24]*Gu1[9] + Gx1[34]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[54]*Gu1[21] + Gx1[64]*Gu1[25] + Gx1[74]*Gu1[29] + Gx1[84]*Gu1[33] + Gx1[94]*Gu1[37];
Gu2[18] = + Gx1[4]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[24]*Gu1[10] + Gx1[34]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[54]*Gu1[22] + Gx1[64]*Gu1[26] + Gx1[74]*Gu1[30] + Gx1[84]*Gu1[34] + Gx1[94]*Gu1[38];
Gu2[19] = + Gx1[4]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[24]*Gu1[11] + Gx1[34]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[54]*Gu1[23] + Gx1[64]*Gu1[27] + Gx1[74]*Gu1[31] + Gx1[84]*Gu1[35] + Gx1[94]*Gu1[39];
Gu2[20] = + Gx1[5]*Gu1[0] + Gx1[15]*Gu1[4] + Gx1[25]*Gu1[8] + Gx1[35]*Gu1[12] + Gx1[45]*Gu1[16] + Gx1[55]*Gu1[20] + Gx1[65]*Gu1[24] + Gx1[75]*Gu1[28] + Gx1[85]*Gu1[32] + Gx1[95]*Gu1[36];
Gu2[21] = + Gx1[5]*Gu1[1] + Gx1[15]*Gu1[5] + Gx1[25]*Gu1[9] + Gx1[35]*Gu1[13] + Gx1[45]*Gu1[17] + Gx1[55]*Gu1[21] + Gx1[65]*Gu1[25] + Gx1[75]*Gu1[29] + Gx1[85]*Gu1[33] + Gx1[95]*Gu1[37];
Gu2[22] = + Gx1[5]*Gu1[2] + Gx1[15]*Gu1[6] + Gx1[25]*Gu1[10] + Gx1[35]*Gu1[14] + Gx1[45]*Gu1[18] + Gx1[55]*Gu1[22] + Gx1[65]*Gu1[26] + Gx1[75]*Gu1[30] + Gx1[85]*Gu1[34] + Gx1[95]*Gu1[38];
Gu2[23] = + Gx1[5]*Gu1[3] + Gx1[15]*Gu1[7] + Gx1[25]*Gu1[11] + Gx1[35]*Gu1[15] + Gx1[45]*Gu1[19] + Gx1[55]*Gu1[23] + Gx1[65]*Gu1[27] + Gx1[75]*Gu1[31] + Gx1[85]*Gu1[35] + Gx1[95]*Gu1[39];
Gu2[24] = + Gx1[6]*Gu1[0] + Gx1[16]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[36]*Gu1[12] + Gx1[46]*Gu1[16] + Gx1[56]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[76]*Gu1[28] + Gx1[86]*Gu1[32] + Gx1[96]*Gu1[36];
Gu2[25] = + Gx1[6]*Gu1[1] + Gx1[16]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[36]*Gu1[13] + Gx1[46]*Gu1[17] + Gx1[56]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[76]*Gu1[29] + Gx1[86]*Gu1[33] + Gx1[96]*Gu1[37];
Gu2[26] = + Gx1[6]*Gu1[2] + Gx1[16]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[36]*Gu1[14] + Gx1[46]*Gu1[18] + Gx1[56]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[76]*Gu1[30] + Gx1[86]*Gu1[34] + Gx1[96]*Gu1[38];
Gu2[27] = + Gx1[6]*Gu1[3] + Gx1[16]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[36]*Gu1[15] + Gx1[46]*Gu1[19] + Gx1[56]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[76]*Gu1[31] + Gx1[86]*Gu1[35] + Gx1[96]*Gu1[39];
Gu2[28] = + Gx1[7]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[27]*Gu1[8] + Gx1[37]*Gu1[12] + Gx1[47]*Gu1[16] + Gx1[57]*Gu1[20] + Gx1[67]*Gu1[24] + Gx1[77]*Gu1[28] + Gx1[87]*Gu1[32] + Gx1[97]*Gu1[36];
Gu2[29] = + Gx1[7]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[27]*Gu1[9] + Gx1[37]*Gu1[13] + Gx1[47]*Gu1[17] + Gx1[57]*Gu1[21] + Gx1[67]*Gu1[25] + Gx1[77]*Gu1[29] + Gx1[87]*Gu1[33] + Gx1[97]*Gu1[37];
Gu2[30] = + Gx1[7]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[27]*Gu1[10] + Gx1[37]*Gu1[14] + Gx1[47]*Gu1[18] + Gx1[57]*Gu1[22] + Gx1[67]*Gu1[26] + Gx1[77]*Gu1[30] + Gx1[87]*Gu1[34] + Gx1[97]*Gu1[38];
Gu2[31] = + Gx1[7]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[27]*Gu1[11] + Gx1[37]*Gu1[15] + Gx1[47]*Gu1[19] + Gx1[57]*Gu1[23] + Gx1[67]*Gu1[27] + Gx1[77]*Gu1[31] + Gx1[87]*Gu1[35] + Gx1[97]*Gu1[39];
Gu2[32] = + Gx1[8]*Gu1[0] + Gx1[18]*Gu1[4] + Gx1[28]*Gu1[8] + Gx1[38]*Gu1[12] + Gx1[48]*Gu1[16] + Gx1[58]*Gu1[20] + Gx1[68]*Gu1[24] + Gx1[78]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[98]*Gu1[36];
Gu2[33] = + Gx1[8]*Gu1[1] + Gx1[18]*Gu1[5] + Gx1[28]*Gu1[9] + Gx1[38]*Gu1[13] + Gx1[48]*Gu1[17] + Gx1[58]*Gu1[21] + Gx1[68]*Gu1[25] + Gx1[78]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[98]*Gu1[37];
Gu2[34] = + Gx1[8]*Gu1[2] + Gx1[18]*Gu1[6] + Gx1[28]*Gu1[10] + Gx1[38]*Gu1[14] + Gx1[48]*Gu1[18] + Gx1[58]*Gu1[22] + Gx1[68]*Gu1[26] + Gx1[78]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[98]*Gu1[38];
Gu2[35] = + Gx1[8]*Gu1[3] + Gx1[18]*Gu1[7] + Gx1[28]*Gu1[11] + Gx1[38]*Gu1[15] + Gx1[48]*Gu1[19] + Gx1[58]*Gu1[23] + Gx1[68]*Gu1[27] + Gx1[78]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[98]*Gu1[39];
Gu2[36] = + Gx1[9]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[29]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[49]*Gu1[16] + Gx1[59]*Gu1[20] + Gx1[69]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[89]*Gu1[32] + Gx1[99]*Gu1[36];
Gu2[37] = + Gx1[9]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[29]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[49]*Gu1[17] + Gx1[59]*Gu1[21] + Gx1[69]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[89]*Gu1[33] + Gx1[99]*Gu1[37];
Gu2[38] = + Gx1[9]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[29]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[49]*Gu1[18] + Gx1[59]*Gu1[22] + Gx1[69]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[89]*Gu1[34] + Gx1[99]*Gu1[38];
Gu2[39] = + Gx1[9]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[29]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[49]*Gu1[19] + Gx1[59]*Gu1[23] + Gx1[69]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[89]*Gu1[35] + Gx1[99]*Gu1[39];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[4] + Q11[2]*Gu1[8] + Q11[3]*Gu1[12] + Q11[4]*Gu1[16] + Q11[5]*Gu1[20] + Q11[6]*Gu1[24] + Q11[7]*Gu1[28] + Q11[8]*Gu1[32] + Q11[9]*Gu1[36] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[5] + Q11[2]*Gu1[9] + Q11[3]*Gu1[13] + Q11[4]*Gu1[17] + Q11[5]*Gu1[21] + Q11[6]*Gu1[25] + Q11[7]*Gu1[29] + Q11[8]*Gu1[33] + Q11[9]*Gu1[37] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[6] + Q11[2]*Gu1[10] + Q11[3]*Gu1[14] + Q11[4]*Gu1[18] + Q11[5]*Gu1[22] + Q11[6]*Gu1[26] + Q11[7]*Gu1[30] + Q11[8]*Gu1[34] + Q11[9]*Gu1[38] + Gu2[2];
Gu3[3] = + Q11[0]*Gu1[3] + Q11[1]*Gu1[7] + Q11[2]*Gu1[11] + Q11[3]*Gu1[15] + Q11[4]*Gu1[19] + Q11[5]*Gu1[23] + Q11[6]*Gu1[27] + Q11[7]*Gu1[31] + Q11[8]*Gu1[35] + Q11[9]*Gu1[39] + Gu2[3];
Gu3[4] = + Q11[10]*Gu1[0] + Q11[11]*Gu1[4] + Q11[12]*Gu1[8] + Q11[13]*Gu1[12] + Q11[14]*Gu1[16] + Q11[15]*Gu1[20] + Q11[16]*Gu1[24] + Q11[17]*Gu1[28] + Q11[18]*Gu1[32] + Q11[19]*Gu1[36] + Gu2[4];
Gu3[5] = + Q11[10]*Gu1[1] + Q11[11]*Gu1[5] + Q11[12]*Gu1[9] + Q11[13]*Gu1[13] + Q11[14]*Gu1[17] + Q11[15]*Gu1[21] + Q11[16]*Gu1[25] + Q11[17]*Gu1[29] + Q11[18]*Gu1[33] + Q11[19]*Gu1[37] + Gu2[5];
Gu3[6] = + Q11[10]*Gu1[2] + Q11[11]*Gu1[6] + Q11[12]*Gu1[10] + Q11[13]*Gu1[14] + Q11[14]*Gu1[18] + Q11[15]*Gu1[22] + Q11[16]*Gu1[26] + Q11[17]*Gu1[30] + Q11[18]*Gu1[34] + Q11[19]*Gu1[38] + Gu2[6];
Gu3[7] = + Q11[10]*Gu1[3] + Q11[11]*Gu1[7] + Q11[12]*Gu1[11] + Q11[13]*Gu1[15] + Q11[14]*Gu1[19] + Q11[15]*Gu1[23] + Q11[16]*Gu1[27] + Q11[17]*Gu1[31] + Q11[18]*Gu1[35] + Q11[19]*Gu1[39] + Gu2[7];
Gu3[8] = + Q11[20]*Gu1[0] + Q11[21]*Gu1[4] + Q11[22]*Gu1[8] + Q11[23]*Gu1[12] + Q11[24]*Gu1[16] + Q11[25]*Gu1[20] + Q11[26]*Gu1[24] + Q11[27]*Gu1[28] + Q11[28]*Gu1[32] + Q11[29]*Gu1[36] + Gu2[8];
Gu3[9] = + Q11[20]*Gu1[1] + Q11[21]*Gu1[5] + Q11[22]*Gu1[9] + Q11[23]*Gu1[13] + Q11[24]*Gu1[17] + Q11[25]*Gu1[21] + Q11[26]*Gu1[25] + Q11[27]*Gu1[29] + Q11[28]*Gu1[33] + Q11[29]*Gu1[37] + Gu2[9];
Gu3[10] = + Q11[20]*Gu1[2] + Q11[21]*Gu1[6] + Q11[22]*Gu1[10] + Q11[23]*Gu1[14] + Q11[24]*Gu1[18] + Q11[25]*Gu1[22] + Q11[26]*Gu1[26] + Q11[27]*Gu1[30] + Q11[28]*Gu1[34] + Q11[29]*Gu1[38] + Gu2[10];
Gu3[11] = + Q11[20]*Gu1[3] + Q11[21]*Gu1[7] + Q11[22]*Gu1[11] + Q11[23]*Gu1[15] + Q11[24]*Gu1[19] + Q11[25]*Gu1[23] + Q11[26]*Gu1[27] + Q11[27]*Gu1[31] + Q11[28]*Gu1[35] + Q11[29]*Gu1[39] + Gu2[11];
Gu3[12] = + Q11[30]*Gu1[0] + Q11[31]*Gu1[4] + Q11[32]*Gu1[8] + Q11[33]*Gu1[12] + Q11[34]*Gu1[16] + Q11[35]*Gu1[20] + Q11[36]*Gu1[24] + Q11[37]*Gu1[28] + Q11[38]*Gu1[32] + Q11[39]*Gu1[36] + Gu2[12];
Gu3[13] = + Q11[30]*Gu1[1] + Q11[31]*Gu1[5] + Q11[32]*Gu1[9] + Q11[33]*Gu1[13] + Q11[34]*Gu1[17] + Q11[35]*Gu1[21] + Q11[36]*Gu1[25] + Q11[37]*Gu1[29] + Q11[38]*Gu1[33] + Q11[39]*Gu1[37] + Gu2[13];
Gu3[14] = + Q11[30]*Gu1[2] + Q11[31]*Gu1[6] + Q11[32]*Gu1[10] + Q11[33]*Gu1[14] + Q11[34]*Gu1[18] + Q11[35]*Gu1[22] + Q11[36]*Gu1[26] + Q11[37]*Gu1[30] + Q11[38]*Gu1[34] + Q11[39]*Gu1[38] + Gu2[14];
Gu3[15] = + Q11[30]*Gu1[3] + Q11[31]*Gu1[7] + Q11[32]*Gu1[11] + Q11[33]*Gu1[15] + Q11[34]*Gu1[19] + Q11[35]*Gu1[23] + Q11[36]*Gu1[27] + Q11[37]*Gu1[31] + Q11[38]*Gu1[35] + Q11[39]*Gu1[39] + Gu2[15];
Gu3[16] = + Q11[40]*Gu1[0] + Q11[41]*Gu1[4] + Q11[42]*Gu1[8] + Q11[43]*Gu1[12] + Q11[44]*Gu1[16] + Q11[45]*Gu1[20] + Q11[46]*Gu1[24] + Q11[47]*Gu1[28] + Q11[48]*Gu1[32] + Q11[49]*Gu1[36] + Gu2[16];
Gu3[17] = + Q11[40]*Gu1[1] + Q11[41]*Gu1[5] + Q11[42]*Gu1[9] + Q11[43]*Gu1[13] + Q11[44]*Gu1[17] + Q11[45]*Gu1[21] + Q11[46]*Gu1[25] + Q11[47]*Gu1[29] + Q11[48]*Gu1[33] + Q11[49]*Gu1[37] + Gu2[17];
Gu3[18] = + Q11[40]*Gu1[2] + Q11[41]*Gu1[6] + Q11[42]*Gu1[10] + Q11[43]*Gu1[14] + Q11[44]*Gu1[18] + Q11[45]*Gu1[22] + Q11[46]*Gu1[26] + Q11[47]*Gu1[30] + Q11[48]*Gu1[34] + Q11[49]*Gu1[38] + Gu2[18];
Gu3[19] = + Q11[40]*Gu1[3] + Q11[41]*Gu1[7] + Q11[42]*Gu1[11] + Q11[43]*Gu1[15] + Q11[44]*Gu1[19] + Q11[45]*Gu1[23] + Q11[46]*Gu1[27] + Q11[47]*Gu1[31] + Q11[48]*Gu1[35] + Q11[49]*Gu1[39] + Gu2[19];
Gu3[20] = + Q11[50]*Gu1[0] + Q11[51]*Gu1[4] + Q11[52]*Gu1[8] + Q11[53]*Gu1[12] + Q11[54]*Gu1[16] + Q11[55]*Gu1[20] + Q11[56]*Gu1[24] + Q11[57]*Gu1[28] + Q11[58]*Gu1[32] + Q11[59]*Gu1[36] + Gu2[20];
Gu3[21] = + Q11[50]*Gu1[1] + Q11[51]*Gu1[5] + Q11[52]*Gu1[9] + Q11[53]*Gu1[13] + Q11[54]*Gu1[17] + Q11[55]*Gu1[21] + Q11[56]*Gu1[25] + Q11[57]*Gu1[29] + Q11[58]*Gu1[33] + Q11[59]*Gu1[37] + Gu2[21];
Gu3[22] = + Q11[50]*Gu1[2] + Q11[51]*Gu1[6] + Q11[52]*Gu1[10] + Q11[53]*Gu1[14] + Q11[54]*Gu1[18] + Q11[55]*Gu1[22] + Q11[56]*Gu1[26] + Q11[57]*Gu1[30] + Q11[58]*Gu1[34] + Q11[59]*Gu1[38] + Gu2[22];
Gu3[23] = + Q11[50]*Gu1[3] + Q11[51]*Gu1[7] + Q11[52]*Gu1[11] + Q11[53]*Gu1[15] + Q11[54]*Gu1[19] + Q11[55]*Gu1[23] + Q11[56]*Gu1[27] + Q11[57]*Gu1[31] + Q11[58]*Gu1[35] + Q11[59]*Gu1[39] + Gu2[23];
Gu3[24] = + Q11[60]*Gu1[0] + Q11[61]*Gu1[4] + Q11[62]*Gu1[8] + Q11[63]*Gu1[12] + Q11[64]*Gu1[16] + Q11[65]*Gu1[20] + Q11[66]*Gu1[24] + Q11[67]*Gu1[28] + Q11[68]*Gu1[32] + Q11[69]*Gu1[36] + Gu2[24];
Gu3[25] = + Q11[60]*Gu1[1] + Q11[61]*Gu1[5] + Q11[62]*Gu1[9] + Q11[63]*Gu1[13] + Q11[64]*Gu1[17] + Q11[65]*Gu1[21] + Q11[66]*Gu1[25] + Q11[67]*Gu1[29] + Q11[68]*Gu1[33] + Q11[69]*Gu1[37] + Gu2[25];
Gu3[26] = + Q11[60]*Gu1[2] + Q11[61]*Gu1[6] + Q11[62]*Gu1[10] + Q11[63]*Gu1[14] + Q11[64]*Gu1[18] + Q11[65]*Gu1[22] + Q11[66]*Gu1[26] + Q11[67]*Gu1[30] + Q11[68]*Gu1[34] + Q11[69]*Gu1[38] + Gu2[26];
Gu3[27] = + Q11[60]*Gu1[3] + Q11[61]*Gu1[7] + Q11[62]*Gu1[11] + Q11[63]*Gu1[15] + Q11[64]*Gu1[19] + Q11[65]*Gu1[23] + Q11[66]*Gu1[27] + Q11[67]*Gu1[31] + Q11[68]*Gu1[35] + Q11[69]*Gu1[39] + Gu2[27];
Gu3[28] = + Q11[70]*Gu1[0] + Q11[71]*Gu1[4] + Q11[72]*Gu1[8] + Q11[73]*Gu1[12] + Q11[74]*Gu1[16] + Q11[75]*Gu1[20] + Q11[76]*Gu1[24] + Q11[77]*Gu1[28] + Q11[78]*Gu1[32] + Q11[79]*Gu1[36] + Gu2[28];
Gu3[29] = + Q11[70]*Gu1[1] + Q11[71]*Gu1[5] + Q11[72]*Gu1[9] + Q11[73]*Gu1[13] + Q11[74]*Gu1[17] + Q11[75]*Gu1[21] + Q11[76]*Gu1[25] + Q11[77]*Gu1[29] + Q11[78]*Gu1[33] + Q11[79]*Gu1[37] + Gu2[29];
Gu3[30] = + Q11[70]*Gu1[2] + Q11[71]*Gu1[6] + Q11[72]*Gu1[10] + Q11[73]*Gu1[14] + Q11[74]*Gu1[18] + Q11[75]*Gu1[22] + Q11[76]*Gu1[26] + Q11[77]*Gu1[30] + Q11[78]*Gu1[34] + Q11[79]*Gu1[38] + Gu2[30];
Gu3[31] = + Q11[70]*Gu1[3] + Q11[71]*Gu1[7] + Q11[72]*Gu1[11] + Q11[73]*Gu1[15] + Q11[74]*Gu1[19] + Q11[75]*Gu1[23] + Q11[76]*Gu1[27] + Q11[77]*Gu1[31] + Q11[78]*Gu1[35] + Q11[79]*Gu1[39] + Gu2[31];
Gu3[32] = + Q11[80]*Gu1[0] + Q11[81]*Gu1[4] + Q11[82]*Gu1[8] + Q11[83]*Gu1[12] + Q11[84]*Gu1[16] + Q11[85]*Gu1[20] + Q11[86]*Gu1[24] + Q11[87]*Gu1[28] + Q11[88]*Gu1[32] + Q11[89]*Gu1[36] + Gu2[32];
Gu3[33] = + Q11[80]*Gu1[1] + Q11[81]*Gu1[5] + Q11[82]*Gu1[9] + Q11[83]*Gu1[13] + Q11[84]*Gu1[17] + Q11[85]*Gu1[21] + Q11[86]*Gu1[25] + Q11[87]*Gu1[29] + Q11[88]*Gu1[33] + Q11[89]*Gu1[37] + Gu2[33];
Gu3[34] = + Q11[80]*Gu1[2] + Q11[81]*Gu1[6] + Q11[82]*Gu1[10] + Q11[83]*Gu1[14] + Q11[84]*Gu1[18] + Q11[85]*Gu1[22] + Q11[86]*Gu1[26] + Q11[87]*Gu1[30] + Q11[88]*Gu1[34] + Q11[89]*Gu1[38] + Gu2[34];
Gu3[35] = + Q11[80]*Gu1[3] + Q11[81]*Gu1[7] + Q11[82]*Gu1[11] + Q11[83]*Gu1[15] + Q11[84]*Gu1[19] + Q11[85]*Gu1[23] + Q11[86]*Gu1[27] + Q11[87]*Gu1[31] + Q11[88]*Gu1[35] + Q11[89]*Gu1[39] + Gu2[35];
Gu3[36] = + Q11[90]*Gu1[0] + Q11[91]*Gu1[4] + Q11[92]*Gu1[8] + Q11[93]*Gu1[12] + Q11[94]*Gu1[16] + Q11[95]*Gu1[20] + Q11[96]*Gu1[24] + Q11[97]*Gu1[28] + Q11[98]*Gu1[32] + Q11[99]*Gu1[36] + Gu2[36];
Gu3[37] = + Q11[90]*Gu1[1] + Q11[91]*Gu1[5] + Q11[92]*Gu1[9] + Q11[93]*Gu1[13] + Q11[94]*Gu1[17] + Q11[95]*Gu1[21] + Q11[96]*Gu1[25] + Q11[97]*Gu1[29] + Q11[98]*Gu1[33] + Q11[99]*Gu1[37] + Gu2[37];
Gu3[38] = + Q11[90]*Gu1[2] + Q11[91]*Gu1[6] + Q11[92]*Gu1[10] + Q11[93]*Gu1[14] + Q11[94]*Gu1[18] + Q11[95]*Gu1[22] + Q11[96]*Gu1[26] + Q11[97]*Gu1[30] + Q11[98]*Gu1[34] + Q11[99]*Gu1[38] + Gu2[38];
Gu3[39] = + Q11[90]*Gu1[3] + Q11[91]*Gu1[7] + Q11[92]*Gu1[11] + Q11[93]*Gu1[15] + Q11[94]*Gu1[19] + Q11[95]*Gu1[23] + Q11[96]*Gu1[27] + Q11[97]*Gu1[31] + Q11[98]*Gu1[35] + Q11[99]*Gu1[39] + Gu2[39];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[10]*w11[1] + Gx1[20]*w11[2] + Gx1[30]*w11[3] + Gx1[40]*w11[4] + Gx1[50]*w11[5] + Gx1[60]*w11[6] + Gx1[70]*w11[7] + Gx1[80]*w11[8] + Gx1[90]*w11[9] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[11]*w11[1] + Gx1[21]*w11[2] + Gx1[31]*w11[3] + Gx1[41]*w11[4] + Gx1[51]*w11[5] + Gx1[61]*w11[6] + Gx1[71]*w11[7] + Gx1[81]*w11[8] + Gx1[91]*w11[9] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[12]*w11[1] + Gx1[22]*w11[2] + Gx1[32]*w11[3] + Gx1[42]*w11[4] + Gx1[52]*w11[5] + Gx1[62]*w11[6] + Gx1[72]*w11[7] + Gx1[82]*w11[8] + Gx1[92]*w11[9] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[13]*w11[1] + Gx1[23]*w11[2] + Gx1[33]*w11[3] + Gx1[43]*w11[4] + Gx1[53]*w11[5] + Gx1[63]*w11[6] + Gx1[73]*w11[7] + Gx1[83]*w11[8] + Gx1[93]*w11[9] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[14]*w11[1] + Gx1[24]*w11[2] + Gx1[34]*w11[3] + Gx1[44]*w11[4] + Gx1[54]*w11[5] + Gx1[64]*w11[6] + Gx1[74]*w11[7] + Gx1[84]*w11[8] + Gx1[94]*w11[9] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[15]*w11[1] + Gx1[25]*w11[2] + Gx1[35]*w11[3] + Gx1[45]*w11[4] + Gx1[55]*w11[5] + Gx1[65]*w11[6] + Gx1[75]*w11[7] + Gx1[85]*w11[8] + Gx1[95]*w11[9] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[16]*w11[1] + Gx1[26]*w11[2] + Gx1[36]*w11[3] + Gx1[46]*w11[4] + Gx1[56]*w11[5] + Gx1[66]*w11[6] + Gx1[76]*w11[7] + Gx1[86]*w11[8] + Gx1[96]*w11[9] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[17]*w11[1] + Gx1[27]*w11[2] + Gx1[37]*w11[3] + Gx1[47]*w11[4] + Gx1[57]*w11[5] + Gx1[67]*w11[6] + Gx1[77]*w11[7] + Gx1[87]*w11[8] + Gx1[97]*w11[9] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[18]*w11[1] + Gx1[28]*w11[2] + Gx1[38]*w11[3] + Gx1[48]*w11[4] + Gx1[58]*w11[5] + Gx1[68]*w11[6] + Gx1[78]*w11[7] + Gx1[88]*w11[8] + Gx1[98]*w11[9] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[19]*w11[1] + Gx1[29]*w11[2] + Gx1[39]*w11[3] + Gx1[49]*w11[4] + Gx1[59]*w11[5] + Gx1[69]*w11[6] + Gx1[79]*w11[7] + Gx1[89]*w11[8] + Gx1[99]*w11[9] + w12[9];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7] + Gu1[32]*w11[8] + Gu1[36]*w11[9];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7] + Gu1[33]*w11[8] + Gu1[37]*w11[9];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7] + Gu1[34]*w11[8] + Gu1[38]*w11[9];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7] + Gu1[35]*w11[8] + Gu1[39]*w11[9];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + Q11[9]*w11[9] + w12[0];
w13[1] = + Q11[10]*w11[0] + Q11[11]*w11[1] + Q11[12]*w11[2] + Q11[13]*w11[3] + Q11[14]*w11[4] + Q11[15]*w11[5] + Q11[16]*w11[6] + Q11[17]*w11[7] + Q11[18]*w11[8] + Q11[19]*w11[9] + w12[1];
w13[2] = + Q11[20]*w11[0] + Q11[21]*w11[1] + Q11[22]*w11[2] + Q11[23]*w11[3] + Q11[24]*w11[4] + Q11[25]*w11[5] + Q11[26]*w11[6] + Q11[27]*w11[7] + Q11[28]*w11[8] + Q11[29]*w11[9] + w12[2];
w13[3] = + Q11[30]*w11[0] + Q11[31]*w11[1] + Q11[32]*w11[2] + Q11[33]*w11[3] + Q11[34]*w11[4] + Q11[35]*w11[5] + Q11[36]*w11[6] + Q11[37]*w11[7] + Q11[38]*w11[8] + Q11[39]*w11[9] + w12[3];
w13[4] = + Q11[40]*w11[0] + Q11[41]*w11[1] + Q11[42]*w11[2] + Q11[43]*w11[3] + Q11[44]*w11[4] + Q11[45]*w11[5] + Q11[46]*w11[6] + Q11[47]*w11[7] + Q11[48]*w11[8] + Q11[49]*w11[9] + w12[4];
w13[5] = + Q11[50]*w11[0] + Q11[51]*w11[1] + Q11[52]*w11[2] + Q11[53]*w11[3] + Q11[54]*w11[4] + Q11[55]*w11[5] + Q11[56]*w11[6] + Q11[57]*w11[7] + Q11[58]*w11[8] + Q11[59]*w11[9] + w12[5];
w13[6] = + Q11[60]*w11[0] + Q11[61]*w11[1] + Q11[62]*w11[2] + Q11[63]*w11[3] + Q11[64]*w11[4] + Q11[65]*w11[5] + Q11[66]*w11[6] + Q11[67]*w11[7] + Q11[68]*w11[8] + Q11[69]*w11[9] + w12[6];
w13[7] = + Q11[70]*w11[0] + Q11[71]*w11[1] + Q11[72]*w11[2] + Q11[73]*w11[3] + Q11[74]*w11[4] + Q11[75]*w11[5] + Q11[76]*w11[6] + Q11[77]*w11[7] + Q11[78]*w11[8] + Q11[79]*w11[9] + w12[7];
w13[8] = + Q11[80]*w11[0] + Q11[81]*w11[1] + Q11[82]*w11[2] + Q11[83]*w11[3] + Q11[84]*w11[4] + Q11[85]*w11[5] + Q11[86]*w11[6] + Q11[87]*w11[7] + Q11[88]*w11[8] + Q11[89]*w11[9] + w12[8];
w13[9] = + Q11[90]*w11[0] + Q11[91]*w11[1] + Q11[92]*w11[2] + Q11[93]*w11[3] + Q11[94]*w11[4] + Q11[95]*w11[5] + Q11[96]*w11[6] + Q11[97]*w11[7] + Q11[98]*w11[8] + Q11[99]*w11[9] + w12[9];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9];
w12[1] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4] + Gx1[15]*w11[5] + Gx1[16]*w11[6] + Gx1[17]*w11[7] + Gx1[18]*w11[8] + Gx1[19]*w11[9];
w12[2] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4] + Gx1[25]*w11[5] + Gx1[26]*w11[6] + Gx1[27]*w11[7] + Gx1[28]*w11[8] + Gx1[29]*w11[9];
w12[3] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5] + Gx1[36]*w11[6] + Gx1[37]*w11[7] + Gx1[38]*w11[8] + Gx1[39]*w11[9];
w12[4] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7] + Gx1[48]*w11[8] + Gx1[49]*w11[9];
w12[5] += + Gx1[50]*w11[0] + Gx1[51]*w11[1] + Gx1[52]*w11[2] + Gx1[53]*w11[3] + Gx1[54]*w11[4] + Gx1[55]*w11[5] + Gx1[56]*w11[6] + Gx1[57]*w11[7] + Gx1[58]*w11[8] + Gx1[59]*w11[9];
w12[6] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9];
w12[7] += + Gx1[70]*w11[0] + Gx1[71]*w11[1] + Gx1[72]*w11[2] + Gx1[73]*w11[3] + Gx1[74]*w11[4] + Gx1[75]*w11[5] + Gx1[76]*w11[6] + Gx1[77]*w11[7] + Gx1[78]*w11[8] + Gx1[79]*w11[9];
w12[8] += + Gx1[80]*w11[0] + Gx1[81]*w11[1] + Gx1[82]*w11[2] + Gx1[83]*w11[3] + Gx1[84]*w11[4] + Gx1[85]*w11[5] + Gx1[86]*w11[6] + Gx1[87]*w11[7] + Gx1[88]*w11[8] + Gx1[89]*w11[9];
w12[9] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9];
w12[1] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4] + Gx1[15]*w11[5] + Gx1[16]*w11[6] + Gx1[17]*w11[7] + Gx1[18]*w11[8] + Gx1[19]*w11[9];
w12[2] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4] + Gx1[25]*w11[5] + Gx1[26]*w11[6] + Gx1[27]*w11[7] + Gx1[28]*w11[8] + Gx1[29]*w11[9];
w12[3] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5] + Gx1[36]*w11[6] + Gx1[37]*w11[7] + Gx1[38]*w11[8] + Gx1[39]*w11[9];
w12[4] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7] + Gx1[48]*w11[8] + Gx1[49]*w11[9];
w12[5] += + Gx1[50]*w11[0] + Gx1[51]*w11[1] + Gx1[52]*w11[2] + Gx1[53]*w11[3] + Gx1[54]*w11[4] + Gx1[55]*w11[5] + Gx1[56]*w11[6] + Gx1[57]*w11[7] + Gx1[58]*w11[8] + Gx1[59]*w11[9];
w12[6] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9];
w12[7] += + Gx1[70]*w11[0] + Gx1[71]*w11[1] + Gx1[72]*w11[2] + Gx1[73]*w11[3] + Gx1[74]*w11[4] + Gx1[75]*w11[5] + Gx1[76]*w11[6] + Gx1[77]*w11[7] + Gx1[78]*w11[8] + Gx1[79]*w11[9];
w12[8] += + Gx1[80]*w11[0] + Gx1[81]*w11[1] + Gx1[82]*w11[2] + Gx1[83]*w11[3] + Gx1[84]*w11[4] + Gx1[85]*w11[5] + Gx1[86]*w11[6] + Gx1[87]*w11[7] + Gx1[88]*w11[8] + Gx1[89]*w11[9];
w12[9] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
w12[2] += + Gu1[8]*U1[0] + Gu1[9]*U1[1] + Gu1[10]*U1[2] + Gu1[11]*U1[3];
w12[3] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3];
w12[4] += + Gu1[16]*U1[0] + Gu1[17]*U1[1] + Gu1[18]*U1[2] + Gu1[19]*U1[3];
w12[5] += + Gu1[20]*U1[0] + Gu1[21]*U1[1] + Gu1[22]*U1[2] + Gu1[23]*U1[3];
w12[6] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3];
w12[7] += + Gu1[28]*U1[0] + Gu1[29]*U1[1] + Gu1[30]*U1[2] + Gu1[31]*U1[3];
w12[8] += + Gu1[32]*U1[0] + Gu1[33]*U1[1] + Gu1[34]*U1[2] + Gu1[35]*U1[3];
w12[9] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2] + Gu1[39]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13];
RDy1[1] = + R2[14]*Dy1[0] + R2[15]*Dy1[1] + R2[16]*Dy1[2] + R2[17]*Dy1[3] + R2[18]*Dy1[4] + R2[19]*Dy1[5] + R2[20]*Dy1[6] + R2[21]*Dy1[7] + R2[22]*Dy1[8] + R2[23]*Dy1[9] + R2[24]*Dy1[10] + R2[25]*Dy1[11] + R2[26]*Dy1[12] + R2[27]*Dy1[13];
RDy1[2] = + R2[28]*Dy1[0] + R2[29]*Dy1[1] + R2[30]*Dy1[2] + R2[31]*Dy1[3] + R2[32]*Dy1[4] + R2[33]*Dy1[5] + R2[34]*Dy1[6] + R2[35]*Dy1[7] + R2[36]*Dy1[8] + R2[37]*Dy1[9] + R2[38]*Dy1[10] + R2[39]*Dy1[11] + R2[40]*Dy1[12] + R2[41]*Dy1[13];
RDy1[3] = + R2[42]*Dy1[0] + R2[43]*Dy1[1] + R2[44]*Dy1[2] + R2[45]*Dy1[3] + R2[46]*Dy1[4] + R2[47]*Dy1[5] + R2[48]*Dy1[6] + R2[49]*Dy1[7] + R2[50]*Dy1[8] + R2[51]*Dy1[9] + R2[52]*Dy1[10] + R2[53]*Dy1[11] + R2[54]*Dy1[12] + R2[55]*Dy1[13];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13];
QDy1[1] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6] + Q2[21]*Dy1[7] + Q2[22]*Dy1[8] + Q2[23]*Dy1[9] + Q2[24]*Dy1[10] + Q2[25]*Dy1[11] + Q2[26]*Dy1[12] + Q2[27]*Dy1[13];
QDy1[2] = + Q2[28]*Dy1[0] + Q2[29]*Dy1[1] + Q2[30]*Dy1[2] + Q2[31]*Dy1[3] + Q2[32]*Dy1[4] + Q2[33]*Dy1[5] + Q2[34]*Dy1[6] + Q2[35]*Dy1[7] + Q2[36]*Dy1[8] + Q2[37]*Dy1[9] + Q2[38]*Dy1[10] + Q2[39]*Dy1[11] + Q2[40]*Dy1[12] + Q2[41]*Dy1[13];
QDy1[3] = + Q2[42]*Dy1[0] + Q2[43]*Dy1[1] + Q2[44]*Dy1[2] + Q2[45]*Dy1[3] + Q2[46]*Dy1[4] + Q2[47]*Dy1[5] + Q2[48]*Dy1[6] + Q2[49]*Dy1[7] + Q2[50]*Dy1[8] + Q2[51]*Dy1[9] + Q2[52]*Dy1[10] + Q2[53]*Dy1[11] + Q2[54]*Dy1[12] + Q2[55]*Dy1[13];
QDy1[4] = + Q2[56]*Dy1[0] + Q2[57]*Dy1[1] + Q2[58]*Dy1[2] + Q2[59]*Dy1[3] + Q2[60]*Dy1[4] + Q2[61]*Dy1[5] + Q2[62]*Dy1[6] + Q2[63]*Dy1[7] + Q2[64]*Dy1[8] + Q2[65]*Dy1[9] + Q2[66]*Dy1[10] + Q2[67]*Dy1[11] + Q2[68]*Dy1[12] + Q2[69]*Dy1[13];
QDy1[5] = + Q2[70]*Dy1[0] + Q2[71]*Dy1[1] + Q2[72]*Dy1[2] + Q2[73]*Dy1[3] + Q2[74]*Dy1[4] + Q2[75]*Dy1[5] + Q2[76]*Dy1[6] + Q2[77]*Dy1[7] + Q2[78]*Dy1[8] + Q2[79]*Dy1[9] + Q2[80]*Dy1[10] + Q2[81]*Dy1[11] + Q2[82]*Dy1[12] + Q2[83]*Dy1[13];
QDy1[6] = + Q2[84]*Dy1[0] + Q2[85]*Dy1[1] + Q2[86]*Dy1[2] + Q2[87]*Dy1[3] + Q2[88]*Dy1[4] + Q2[89]*Dy1[5] + Q2[90]*Dy1[6] + Q2[91]*Dy1[7] + Q2[92]*Dy1[8] + Q2[93]*Dy1[9] + Q2[94]*Dy1[10] + Q2[95]*Dy1[11] + Q2[96]*Dy1[12] + Q2[97]*Dy1[13];
QDy1[7] = + Q2[98]*Dy1[0] + Q2[99]*Dy1[1] + Q2[100]*Dy1[2] + Q2[101]*Dy1[3] + Q2[102]*Dy1[4] + Q2[103]*Dy1[5] + Q2[104]*Dy1[6] + Q2[105]*Dy1[7] + Q2[106]*Dy1[8] + Q2[107]*Dy1[9] + Q2[108]*Dy1[10] + Q2[109]*Dy1[11] + Q2[110]*Dy1[12] + Q2[111]*Dy1[13];
QDy1[8] = + Q2[112]*Dy1[0] + Q2[113]*Dy1[1] + Q2[114]*Dy1[2] + Q2[115]*Dy1[3] + Q2[116]*Dy1[4] + Q2[117]*Dy1[5] + Q2[118]*Dy1[6] + Q2[119]*Dy1[7] + Q2[120]*Dy1[8] + Q2[121]*Dy1[9] + Q2[122]*Dy1[10] + Q2[123]*Dy1[11] + Q2[124]*Dy1[12] + Q2[125]*Dy1[13];
QDy1[9] = + Q2[126]*Dy1[0] + Q2[127]*Dy1[1] + Q2[128]*Dy1[2] + Q2[129]*Dy1[3] + Q2[130]*Dy1[4] + Q2[131]*Dy1[5] + Q2[132]*Dy1[6] + Q2[133]*Dy1[7] + Q2[134]*Dy1[8] + Q2[135]*Dy1[9] + Q2[136]*Dy1[10] + Q2[137]*Dy1[11] + Q2[138]*Dy1[12] + Q2[139]*Dy1[13];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 80) + (col * 4)] = + Hx[0]*E[0] + Hx[1]*E[4] + Hx[2]*E[8] + Hx[3]*E[12] + Hx[4]*E[16] + Hx[5]*E[20] + Hx[6]*E[24] + Hx[7]*E[28] + Hx[8]*E[32] + Hx[9]*E[36];
acadoWorkspace.A[(row * 80) + (col * 4 + 1)] = + Hx[0]*E[1] + Hx[1]*E[5] + Hx[2]*E[9] + Hx[3]*E[13] + Hx[4]*E[17] + Hx[5]*E[21] + Hx[6]*E[25] + Hx[7]*E[29] + Hx[8]*E[33] + Hx[9]*E[37];
acadoWorkspace.A[(row * 80) + (col * 4 + 2)] = + Hx[0]*E[2] + Hx[1]*E[6] + Hx[2]*E[10] + Hx[3]*E[14] + Hx[4]*E[18] + Hx[5]*E[22] + Hx[6]*E[26] + Hx[7]*E[30] + Hx[8]*E[34] + Hx[9]*E[38];
acadoWorkspace.A[(row * 80) + (col * 4 + 3)] = + Hx[0]*E[3] + Hx[1]*E[7] + Hx[2]*E[11] + Hx[3]*E[15] + Hx[4]*E[19] + Hx[5]*E[23] + Hx[6]*E[27] + Hx[7]*E[31] + Hx[8]*E[35] + Hx[9]*E[39];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6] + Hx[7]*tmpd[7] + Hx[8]*tmpd[8] + Hx[9]*tmpd[9];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 100 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.C[ 100 ]), &(acadoWorkspace.C[ 200 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.C[ 200 ]), &(acadoWorkspace.C[ 300 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.C[ 300 ]), &(acadoWorkspace.C[ 400 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.C[ 400 ]), &(acadoWorkspace.C[ 500 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.C[ 500 ]), &(acadoWorkspace.C[ 600 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.C[ 600 ]), &(acadoWorkspace.C[ 700 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.C[ 700 ]), &(acadoWorkspace.C[ 800 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.C[ 800 ]), &(acadoWorkspace.C[ 900 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.C[ 900 ]), &(acadoWorkspace.C[ 1000 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.C[ 1000 ]), &(acadoWorkspace.C[ 1100 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.C[ 1100 ]), &(acadoWorkspace.C[ 1200 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.C[ 1200 ]), &(acadoWorkspace.C[ 1300 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.C[ 1300 ]), &(acadoWorkspace.C[ 1400 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1500 ]), &(acadoWorkspace.C[ 1400 ]), &(acadoWorkspace.C[ 1500 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1600 ]), &(acadoWorkspace.C[ 1500 ]), &(acadoWorkspace.C[ 1600 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1700 ]), &(acadoWorkspace.C[ 1600 ]), &(acadoWorkspace.C[ 1700 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1800 ]), &(acadoWorkspace.C[ 1700 ]), &(acadoWorkspace.C[ 1800 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1900 ]), &(acadoWorkspace.C[ 1800 ]), &(acadoWorkspace.C[ 1900 ]) );
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 40 ]), &(acadoWorkspace.E[ lRun3 * 40 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (10)) * (10)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (10)) * (4)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (10)) * (4)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (10)) * (4)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 40 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 100 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (10)) * (4)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 16 ]), &(acadoWorkspace.evGu[ lRun2 * 40 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

for (lRun2 = 0; lRun2 < 200; ++lRun2)
acadoWorkspace.sbar[lRun2 + 10] = acadoWorkspace.d[lRun2];


for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.conValueIn[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.conValueIn[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.conValueIn[9] = acadoVariables.x[lRun1 * 10 + 9];
acadoWorkspace.conValueIn[10] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.conValueIn[11] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[12] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[13] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun1 * 10];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun1 * 10 + 1];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun1 * 10 + 2];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun1 * 10 + 3];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun1 * 10 + 4];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun1 * 10 + 5];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun1 * 10 + 6];
acadoWorkspace.conValueIn[21] = acadoVariables.od[lRun1 * 10 + 7];
acadoWorkspace.conValueIn[22] = acadoVariables.od[lRun1 * 10 + 8];
acadoWorkspace.conValueIn[23] = acadoVariables.od[lRun1 * 10 + 9];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 10] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 10 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 10 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 10 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 10 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 10 + 5] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 10 + 6] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 10 + 7] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 10 + 8] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 10 + 9] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[14];
}



acado_multHxE( &(acadoWorkspace.evHx[ 10 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.E[ 40 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.E[ 800 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 80 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 840 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 1560 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 120 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 880 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 1600 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 2280 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.E[ 160 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.E[ 920 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.E[ 1640 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.E[ 2320 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.E[ 2960 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 200 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 960 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 1680 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 2360 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 3000 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 3600 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.E[ 240 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.E[ 1000 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.E[ 1720 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.E[ 2400 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.E[ 3040 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.E[ 3640 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.E[ 4200 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 280 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 1040 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 1760 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 2440 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 3080 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 3680 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 4240 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 4760 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 320 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1080 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1800 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 2480 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 3120 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 3720 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 4280 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 4800 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 5280 ]), 9, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 360 ]), 10, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 1120 ]), 10, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 1840 ]), 10, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 2520 ]), 10, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 3160 ]), 10, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 3760 ]), 10, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 4320 ]), 10, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 4840 ]), 10, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 5320 ]), 10, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 5760 ]), 10, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 400 ]), 11, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 1160 ]), 11, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 1880 ]), 11, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 2560 ]), 11, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 3200 ]), 11, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 3800 ]), 11, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 4360 ]), 11, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 4880 ]), 11, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 5360 ]), 11, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 5800 ]), 11, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.E[ 6200 ]), 11, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 440 ]), 12, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 1200 ]), 12, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 1920 ]), 12, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 2600 ]), 12, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 3240 ]), 12, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 3840 ]), 12, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 4400 ]), 12, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 4920 ]), 12, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 5400 ]), 12, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 5840 ]), 12, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 6240 ]), 12, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 6600 ]), 12, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 480 ]), 13, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 1240 ]), 13, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 1960 ]), 13, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 2640 ]), 13, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 3280 ]), 13, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 3880 ]), 13, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 4440 ]), 13, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 4960 ]), 13, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 5440 ]), 13, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 5880 ]), 13, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 6280 ]), 13, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 6640 ]), 13, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.E[ 6960 ]), 13, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 520 ]), 14, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 1280 ]), 14, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 2000 ]), 14, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 2680 ]), 14, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 3320 ]), 14, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 3920 ]), 14, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 4480 ]), 14, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 5000 ]), 14, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 5480 ]), 14, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 5920 ]), 14, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 6320 ]), 14, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 6680 ]), 14, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 7000 ]), 14, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 7280 ]), 14, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 560 ]), 15, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 1320 ]), 15, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 2040 ]), 15, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 2720 ]), 15, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 3360 ]), 15, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 3960 ]), 15, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 4520 ]), 15, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 5040 ]), 15, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 5520 ]), 15, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 5960 ]), 15, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 6360 ]), 15, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 6720 ]), 15, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 7040 ]), 15, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 7320 ]), 15, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 7560 ]), 15, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 600 ]), 16, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 1360 ]), 16, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 2080 ]), 16, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 2760 ]), 16, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 3400 ]), 16, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 4000 ]), 16, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 4560 ]), 16, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 5080 ]), 16, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 5560 ]), 16, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 6000 ]), 16, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 6400 ]), 16, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 6760 ]), 16, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 7080 ]), 16, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 7360 ]), 16, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 7600 ]), 16, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 7800 ]), 16, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 640 ]), 17, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 1400 ]), 17, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 2120 ]), 17, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 2800 ]), 17, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 3440 ]), 17, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 4040 ]), 17, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 4600 ]), 17, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 5120 ]), 17, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 5600 ]), 17, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 6040 ]), 17, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 6440 ]), 17, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 6800 ]), 17, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 7120 ]), 17, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 7400 ]), 17, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 7640 ]), 17, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 7840 ]), 17, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.E[ 8000 ]), 17, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 680 ]), 18, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 1440 ]), 18, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 2160 ]), 18, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 2840 ]), 18, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 3480 ]), 18, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 4080 ]), 18, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 4640 ]), 18, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 5160 ]), 18, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 5640 ]), 18, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 6080 ]), 18, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 6480 ]), 18, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 6840 ]), 18, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 7160 ]), 18, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 7440 ]), 18, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 7680 ]), 18, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 7880 ]), 18, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 8040 ]), 18, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 8160 ]), 18, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 720 ]), 19, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 1480 ]), 19, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 2200 ]), 19, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 2880 ]), 19, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 3520 ]), 19, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 4120 ]), 19, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 4680 ]), 19, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 5200 ]), 19, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 5680 ]), 19, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 6120 ]), 19, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 6520 ]), 19, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 6880 ]), 19, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 7200 ]), 19, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 7480 ]), 19, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 7720 ]), 19, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 7920 ]), 19, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 8080 ]), 19, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 8200 ]), 19, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.E[ 8280 ]), 19, 18 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3] = acadoWorkspace.evHu[3];
acadoWorkspace.A[84] = acadoWorkspace.evHu[4];
acadoWorkspace.A[85] = acadoWorkspace.evHu[5];
acadoWorkspace.A[86] = acadoWorkspace.evHu[6];
acadoWorkspace.A[87] = acadoWorkspace.evHu[7];
acadoWorkspace.A[168] = acadoWorkspace.evHu[8];
acadoWorkspace.A[169] = acadoWorkspace.evHu[9];
acadoWorkspace.A[170] = acadoWorkspace.evHu[10];
acadoWorkspace.A[171] = acadoWorkspace.evHu[11];
acadoWorkspace.A[252] = acadoWorkspace.evHu[12];
acadoWorkspace.A[253] = acadoWorkspace.evHu[13];
acadoWorkspace.A[254] = acadoWorkspace.evHu[14];
acadoWorkspace.A[255] = acadoWorkspace.evHu[15];
acadoWorkspace.A[336] = acadoWorkspace.evHu[16];
acadoWorkspace.A[337] = acadoWorkspace.evHu[17];
acadoWorkspace.A[338] = acadoWorkspace.evHu[18];
acadoWorkspace.A[339] = acadoWorkspace.evHu[19];
acadoWorkspace.A[420] = acadoWorkspace.evHu[20];
acadoWorkspace.A[421] = acadoWorkspace.evHu[21];
acadoWorkspace.A[422] = acadoWorkspace.evHu[22];
acadoWorkspace.A[423] = acadoWorkspace.evHu[23];
acadoWorkspace.A[504] = acadoWorkspace.evHu[24];
acadoWorkspace.A[505] = acadoWorkspace.evHu[25];
acadoWorkspace.A[506] = acadoWorkspace.evHu[26];
acadoWorkspace.A[507] = acadoWorkspace.evHu[27];
acadoWorkspace.A[588] = acadoWorkspace.evHu[28];
acadoWorkspace.A[589] = acadoWorkspace.evHu[29];
acadoWorkspace.A[590] = acadoWorkspace.evHu[30];
acadoWorkspace.A[591] = acadoWorkspace.evHu[31];
acadoWorkspace.A[672] = acadoWorkspace.evHu[32];
acadoWorkspace.A[673] = acadoWorkspace.evHu[33];
acadoWorkspace.A[674] = acadoWorkspace.evHu[34];
acadoWorkspace.A[675] = acadoWorkspace.evHu[35];
acadoWorkspace.A[756] = acadoWorkspace.evHu[36];
acadoWorkspace.A[757] = acadoWorkspace.evHu[37];
acadoWorkspace.A[758] = acadoWorkspace.evHu[38];
acadoWorkspace.A[759] = acadoWorkspace.evHu[39];
acadoWorkspace.A[840] = acadoWorkspace.evHu[40];
acadoWorkspace.A[841] = acadoWorkspace.evHu[41];
acadoWorkspace.A[842] = acadoWorkspace.evHu[42];
acadoWorkspace.A[843] = acadoWorkspace.evHu[43];
acadoWorkspace.A[924] = acadoWorkspace.evHu[44];
acadoWorkspace.A[925] = acadoWorkspace.evHu[45];
acadoWorkspace.A[926] = acadoWorkspace.evHu[46];
acadoWorkspace.A[927] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1008] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1009] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1010] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1011] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1092] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1093] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1094] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1095] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1176] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1177] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1178] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1179] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1260] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1261] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1262] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1263] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1344] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1345] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1346] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1347] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1428] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1429] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1430] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1431] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1512] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1513] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1514] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1515] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1596] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1597] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1598] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1599] = acadoWorkspace.evHu[79];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - acadoWorkspace.evH[19];

acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - acadoWorkspace.evH[19];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
for (lRun1 = 0; lRun1 < 280; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];
acadoWorkspace.DyN[9] -= acadoVariables.yN[9];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 56 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 112 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 224 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 392 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 448 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 504 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 560 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 616 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 672 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 728 ]), &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 784 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 840 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 896 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 952 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1008 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1064 ]), &(acadoWorkspace.Dy[ 266 ]), &(acadoWorkspace.g[ 76 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 140 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 280 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 560 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 700 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 980 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1120 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1260 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1400 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1540 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.QDy[ 110 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1680 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1820 ]), &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1960 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2100 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 150 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2240 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.QDy[ 160 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2380 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.QDy[ 170 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2520 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2660 ]), &(acadoWorkspace.Dy[ 266 ]), &(acadoWorkspace.QDy[ 190 ]) );

acadoWorkspace.QDy[200] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[201] = + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[202] = + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[203] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[204] = + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[205] = + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[206] = + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[207] = + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[72]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[73]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[74]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[75]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[76]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[77]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[78]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[79]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[208] = + acadoWorkspace.QN2[80]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[81]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[82]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[83]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[84]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[85]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[86]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[87]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[88]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[89]*acadoWorkspace.DyN[9];
acadoWorkspace.QDy[209] = + acadoWorkspace.QN2[90]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[91]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[92]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[93]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[94]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[95]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[96]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[97]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[98]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[99]*acadoWorkspace.DyN[9];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1500 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1600 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 170 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1700 ]), &(acadoWorkspace.sbar[ 170 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1800 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 190 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1900 ]), &(acadoWorkspace.sbar[ 190 ]), &(acadoWorkspace.sbar[ 200 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[200];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[201];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[202];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[203];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[204];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[205];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[206];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[207];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[81]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[82]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[83]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[84]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[85]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[86]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[87]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[88]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[89]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[208];
acadoWorkspace.w1[9] = + acadoWorkspace.QN1[90]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[91]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[92]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[93]*acadoWorkspace.sbar[203] + acadoWorkspace.QN1[94]*acadoWorkspace.sbar[204] + acadoWorkspace.QN1[95]*acadoWorkspace.sbar[205] + acadoWorkspace.QN1[96]*acadoWorkspace.sbar[206] + acadoWorkspace.QN1[97]*acadoWorkspace.sbar[207] + acadoWorkspace.QN1[98]*acadoWorkspace.sbar[208] + acadoWorkspace.QN1[99]*acadoWorkspace.sbar[209] + acadoWorkspace.QDy[209];
acado_macBTw1( &(acadoWorkspace.evGu[ 760 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1900 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 190 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1900 ]), &(acadoWorkspace.sbar[ 190 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1800 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1800 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 680 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1700 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 170 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1700 ]), &(acadoWorkspace.sbar[ 170 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 640 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1600 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 160 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1600 ]), &(acadoWorkspace.sbar[ 160 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 600 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1500 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 150 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1500 ]), &(acadoWorkspace.sbar[ 150 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1400 ]), &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 130 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1300 ]), &(acadoWorkspace.sbar[ 130 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1200 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 110 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1100 ]), &(acadoWorkspace.sbar[ 110 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1000 ]), &(acadoWorkspace.sbar[ 100 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 900 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 70 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.sbar[ 70 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 50 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.sbar[ 50 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];


acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 10 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.sbar[ 170 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.sbar[ 190 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.sbar[lRun1 + 10] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.evGu[ 200 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.evGu[ 280 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.evGu[ 320 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.evGu[ 360 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.evGu[ 400 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.evGu[ 440 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.evGu[ 520 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.evGu[ 560 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1500 ]), &(acadoWorkspace.evGu[ 600 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1600 ]), &(acadoWorkspace.evGu[ 640 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 170 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1700 ]), &(acadoWorkspace.evGu[ 680 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 170 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1800 ]), &(acadoWorkspace.evGu[ 720 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 190 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1900 ]), &(acadoWorkspace.evGu[ 760 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 190 ]), &(acadoWorkspace.sbar[ 200 ]) );
for (lRun1 = 0; lRun1 < 210; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = 1.9620000000000002e+00;
acadoVariables.lbValues[1] = -2.5000000000000000e+00;
acadoVariables.lbValues[2] = -2.5000000000000000e+00;
acadoVariables.lbValues[3] = -1.0000000000000000e-03;
acadoVariables.lbValues[4] = 1.9620000000000002e+00;
acadoVariables.lbValues[5] = -2.5000000000000000e+00;
acadoVariables.lbValues[6] = -2.5000000000000000e+00;
acadoVariables.lbValues[7] = -1.0000000000000000e-03;
acadoVariables.lbValues[8] = 1.9620000000000002e+00;
acadoVariables.lbValues[9] = -2.5000000000000000e+00;
acadoVariables.lbValues[10] = -2.5000000000000000e+00;
acadoVariables.lbValues[11] = -1.0000000000000000e-03;
acadoVariables.lbValues[12] = 1.9620000000000002e+00;
acadoVariables.lbValues[13] = -2.5000000000000000e+00;
acadoVariables.lbValues[14] = -2.5000000000000000e+00;
acadoVariables.lbValues[15] = -1.0000000000000000e-03;
acadoVariables.lbValues[16] = 1.9620000000000002e+00;
acadoVariables.lbValues[17] = -2.5000000000000000e+00;
acadoVariables.lbValues[18] = -2.5000000000000000e+00;
acadoVariables.lbValues[19] = -1.0000000000000000e-03;
acadoVariables.lbValues[20] = 1.9620000000000002e+00;
acadoVariables.lbValues[21] = -2.5000000000000000e+00;
acadoVariables.lbValues[22] = -2.5000000000000000e+00;
acadoVariables.lbValues[23] = -1.0000000000000000e-03;
acadoVariables.lbValues[24] = 1.9620000000000002e+00;
acadoVariables.lbValues[25] = -2.5000000000000000e+00;
acadoVariables.lbValues[26] = -2.5000000000000000e+00;
acadoVariables.lbValues[27] = -1.0000000000000000e-03;
acadoVariables.lbValues[28] = 1.9620000000000002e+00;
acadoVariables.lbValues[29] = -2.5000000000000000e+00;
acadoVariables.lbValues[30] = -2.5000000000000000e+00;
acadoVariables.lbValues[31] = -1.0000000000000000e-03;
acadoVariables.lbValues[32] = 1.9620000000000002e+00;
acadoVariables.lbValues[33] = -2.5000000000000000e+00;
acadoVariables.lbValues[34] = -2.5000000000000000e+00;
acadoVariables.lbValues[35] = -1.0000000000000000e-03;
acadoVariables.lbValues[36] = 1.9620000000000002e+00;
acadoVariables.lbValues[37] = -2.5000000000000000e+00;
acadoVariables.lbValues[38] = -2.5000000000000000e+00;
acadoVariables.lbValues[39] = -1.0000000000000000e-03;
acadoVariables.lbValues[40] = 1.9620000000000002e+00;
acadoVariables.lbValues[41] = -2.5000000000000000e+00;
acadoVariables.lbValues[42] = -2.5000000000000000e+00;
acadoVariables.lbValues[43] = -1.0000000000000000e-03;
acadoVariables.lbValues[44] = 1.9620000000000002e+00;
acadoVariables.lbValues[45] = -2.5000000000000000e+00;
acadoVariables.lbValues[46] = -2.5000000000000000e+00;
acadoVariables.lbValues[47] = -1.0000000000000000e-03;
acadoVariables.lbValues[48] = 1.9620000000000002e+00;
acadoVariables.lbValues[49] = -2.5000000000000000e+00;
acadoVariables.lbValues[50] = -2.5000000000000000e+00;
acadoVariables.lbValues[51] = -1.0000000000000000e-03;
acadoVariables.lbValues[52] = 1.9620000000000002e+00;
acadoVariables.lbValues[53] = -2.5000000000000000e+00;
acadoVariables.lbValues[54] = -2.5000000000000000e+00;
acadoVariables.lbValues[55] = -1.0000000000000000e-03;
acadoVariables.lbValues[56] = 1.9620000000000002e+00;
acadoVariables.lbValues[57] = -2.5000000000000000e+00;
acadoVariables.lbValues[58] = -2.5000000000000000e+00;
acadoVariables.lbValues[59] = -1.0000000000000000e-03;
acadoVariables.lbValues[60] = 1.9620000000000002e+00;
acadoVariables.lbValues[61] = -2.5000000000000000e+00;
acadoVariables.lbValues[62] = -2.5000000000000000e+00;
acadoVariables.lbValues[63] = -1.0000000000000000e-03;
acadoVariables.lbValues[64] = 1.9620000000000002e+00;
acadoVariables.lbValues[65] = -2.5000000000000000e+00;
acadoVariables.lbValues[66] = -2.5000000000000000e+00;
acadoVariables.lbValues[67] = -1.0000000000000000e-03;
acadoVariables.lbValues[68] = 1.9620000000000002e+00;
acadoVariables.lbValues[69] = -2.5000000000000000e+00;
acadoVariables.lbValues[70] = -2.5000000000000000e+00;
acadoVariables.lbValues[71] = -1.0000000000000000e-03;
acadoVariables.lbValues[72] = 1.9620000000000002e+00;
acadoVariables.lbValues[73] = -2.5000000000000000e+00;
acadoVariables.lbValues[74] = -2.5000000000000000e+00;
acadoVariables.lbValues[75] = -1.0000000000000000e-03;
acadoVariables.lbValues[76] = 1.9620000000000002e+00;
acadoVariables.lbValues[77] = -2.5000000000000000e+00;
acadoVariables.lbValues[78] = -2.5000000000000000e+00;
acadoVariables.lbValues[79] = -1.0000000000000000e-03;
acadoVariables.ubValues[0] = 1.9620000000000001e+01;
acadoVariables.ubValues[1] = 2.5000000000000000e+00;
acadoVariables.ubValues[2] = 2.5000000000000000e+00;
acadoVariables.ubValues[3] = 1.0000000000000000e-03;
acadoVariables.ubValues[4] = 1.9620000000000001e+01;
acadoVariables.ubValues[5] = 2.5000000000000000e+00;
acadoVariables.ubValues[6] = 2.5000000000000000e+00;
acadoVariables.ubValues[7] = 1.0000000000000000e-03;
acadoVariables.ubValues[8] = 1.9620000000000001e+01;
acadoVariables.ubValues[9] = 2.5000000000000000e+00;
acadoVariables.ubValues[10] = 2.5000000000000000e+00;
acadoVariables.ubValues[11] = 1.0000000000000000e-03;
acadoVariables.ubValues[12] = 1.9620000000000001e+01;
acadoVariables.ubValues[13] = 2.5000000000000000e+00;
acadoVariables.ubValues[14] = 2.5000000000000000e+00;
acadoVariables.ubValues[15] = 1.0000000000000000e-03;
acadoVariables.ubValues[16] = 1.9620000000000001e+01;
acadoVariables.ubValues[17] = 2.5000000000000000e+00;
acadoVariables.ubValues[18] = 2.5000000000000000e+00;
acadoVariables.ubValues[19] = 1.0000000000000000e-03;
acadoVariables.ubValues[20] = 1.9620000000000001e+01;
acadoVariables.ubValues[21] = 2.5000000000000000e+00;
acadoVariables.ubValues[22] = 2.5000000000000000e+00;
acadoVariables.ubValues[23] = 1.0000000000000000e-03;
acadoVariables.ubValues[24] = 1.9620000000000001e+01;
acadoVariables.ubValues[25] = 2.5000000000000000e+00;
acadoVariables.ubValues[26] = 2.5000000000000000e+00;
acadoVariables.ubValues[27] = 1.0000000000000000e-03;
acadoVariables.ubValues[28] = 1.9620000000000001e+01;
acadoVariables.ubValues[29] = 2.5000000000000000e+00;
acadoVariables.ubValues[30] = 2.5000000000000000e+00;
acadoVariables.ubValues[31] = 1.0000000000000000e-03;
acadoVariables.ubValues[32] = 1.9620000000000001e+01;
acadoVariables.ubValues[33] = 2.5000000000000000e+00;
acadoVariables.ubValues[34] = 2.5000000000000000e+00;
acadoVariables.ubValues[35] = 1.0000000000000000e-03;
acadoVariables.ubValues[36] = 1.9620000000000001e+01;
acadoVariables.ubValues[37] = 2.5000000000000000e+00;
acadoVariables.ubValues[38] = 2.5000000000000000e+00;
acadoVariables.ubValues[39] = 1.0000000000000000e-03;
acadoVariables.ubValues[40] = 1.9620000000000001e+01;
acadoVariables.ubValues[41] = 2.5000000000000000e+00;
acadoVariables.ubValues[42] = 2.5000000000000000e+00;
acadoVariables.ubValues[43] = 1.0000000000000000e-03;
acadoVariables.ubValues[44] = 1.9620000000000001e+01;
acadoVariables.ubValues[45] = 2.5000000000000000e+00;
acadoVariables.ubValues[46] = 2.5000000000000000e+00;
acadoVariables.ubValues[47] = 1.0000000000000000e-03;
acadoVariables.ubValues[48] = 1.9620000000000001e+01;
acadoVariables.ubValues[49] = 2.5000000000000000e+00;
acadoVariables.ubValues[50] = 2.5000000000000000e+00;
acadoVariables.ubValues[51] = 1.0000000000000000e-03;
acadoVariables.ubValues[52] = 1.9620000000000001e+01;
acadoVariables.ubValues[53] = 2.5000000000000000e+00;
acadoVariables.ubValues[54] = 2.5000000000000000e+00;
acadoVariables.ubValues[55] = 1.0000000000000000e-03;
acadoVariables.ubValues[56] = 1.9620000000000001e+01;
acadoVariables.ubValues[57] = 2.5000000000000000e+00;
acadoVariables.ubValues[58] = 2.5000000000000000e+00;
acadoVariables.ubValues[59] = 1.0000000000000000e-03;
acadoVariables.ubValues[60] = 1.9620000000000001e+01;
acadoVariables.ubValues[61] = 2.5000000000000000e+00;
acadoVariables.ubValues[62] = 2.5000000000000000e+00;
acadoVariables.ubValues[63] = 1.0000000000000000e-03;
acadoVariables.ubValues[64] = 1.9620000000000001e+01;
acadoVariables.ubValues[65] = 2.5000000000000000e+00;
acadoVariables.ubValues[66] = 2.5000000000000000e+00;
acadoVariables.ubValues[67] = 1.0000000000000000e-03;
acadoVariables.ubValues[68] = 1.9620000000000001e+01;
acadoVariables.ubValues[69] = 2.5000000000000000e+00;
acadoVariables.ubValues[70] = 2.5000000000000000e+00;
acadoVariables.ubValues[71] = 1.0000000000000000e-03;
acadoVariables.ubValues[72] = 1.9620000000000001e+01;
acadoVariables.ubValues[73] = 2.5000000000000000e+00;
acadoVariables.ubValues[74] = 2.5000000000000000e+00;
acadoVariables.ubValues[75] = 1.0000000000000000e-03;
acadoVariables.ubValues[76] = 1.9620000000000001e+01;
acadoVariables.ubValues[77] = 2.5000000000000000e+00;
acadoVariables.ubValues[78] = 2.5000000000000000e+00;
acadoVariables.ubValues[79] = 1.0000000000000000e-03;
acadoVariables.lbAValues[0] = -1.0000000000000000e+12;
acadoVariables.lbAValues[1] = -1.0000000000000000e+12;
acadoVariables.lbAValues[2] = -1.0000000000000000e+12;
acadoVariables.lbAValues[3] = -1.0000000000000000e+12;
acadoVariables.lbAValues[4] = -1.0000000000000000e+12;
acadoVariables.lbAValues[5] = -1.0000000000000000e+12;
acadoVariables.lbAValues[6] = -1.0000000000000000e+12;
acadoVariables.lbAValues[7] = -1.0000000000000000e+12;
acadoVariables.lbAValues[8] = -1.0000000000000000e+12;
acadoVariables.lbAValues[9] = -1.0000000000000000e+12;
acadoVariables.lbAValues[10] = -1.0000000000000000e+12;
acadoVariables.lbAValues[11] = -1.0000000000000000e+12;
acadoVariables.lbAValues[12] = -1.0000000000000000e+12;
acadoVariables.lbAValues[13] = -1.0000000000000000e+12;
acadoVariables.lbAValues[14] = -1.0000000000000000e+12;
acadoVariables.lbAValues[15] = -1.0000000000000000e+12;
acadoVariables.lbAValues[16] = -1.0000000000000000e+12;
acadoVariables.lbAValues[17] = -1.0000000000000000e+12;
acadoVariables.lbAValues[18] = -1.0000000000000000e+12;
acadoVariables.lbAValues[19] = -1.0000000000000000e+12;
acadoVariables.ubAValues[0] = 1.9620000000000001e+01;
acadoVariables.ubAValues[1] = 1.9620000000000001e+01;
acadoVariables.ubAValues[2] = 1.9620000000000001e+01;
acadoVariables.ubAValues[3] = 1.9620000000000001e+01;
acadoVariables.ubAValues[4] = 1.9620000000000001e+01;
acadoVariables.ubAValues[5] = 1.9620000000000001e+01;
acadoVariables.ubAValues[6] = 1.9620000000000001e+01;
acadoVariables.ubAValues[7] = 1.9620000000000001e+01;
acadoVariables.ubAValues[8] = 1.9620000000000001e+01;
acadoVariables.ubAValues[9] = 1.9620000000000001e+01;
acadoVariables.ubAValues[10] = 1.9620000000000001e+01;
acadoVariables.ubAValues[11] = 1.9620000000000001e+01;
acadoVariables.ubAValues[12] = 1.9620000000000001e+01;
acadoVariables.ubAValues[13] = 1.9620000000000001e+01;
acadoVariables.ubAValues[14] = 1.9620000000000001e+01;
acadoVariables.ubAValues[15] = 1.9620000000000001e+01;
acadoVariables.ubAValues[16] = 1.9620000000000001e+01;
acadoVariables.ubAValues[17] = 1.9620000000000001e+01;
acadoVariables.ubAValues[18] = 1.9620000000000001e+01;
acadoVariables.ubAValues[19] = 1.9620000000000001e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 10];
state[1] = acadoVariables.x[index * 10 + 1];
state[2] = acadoVariables.x[index * 10 + 2];
state[3] = acadoVariables.x[index * 10 + 3];
state[4] = acadoVariables.x[index * 10 + 4];
state[5] = acadoVariables.x[index * 10 + 5];
state[6] = acadoVariables.x[index * 10 + 6];
state[7] = acadoVariables.x[index * 10 + 7];
state[8] = acadoVariables.x[index * 10 + 8];
state[9] = acadoVariables.x[index * 10 + 9];
state[150] = acadoVariables.u[index * 4];
state[151] = acadoVariables.u[index * 4 + 1];
state[152] = acadoVariables.u[index * 4 + 2];
state[153] = acadoVariables.u[index * 4 + 3];
state[154] = acadoVariables.od[index * 10];
state[155] = acadoVariables.od[index * 10 + 1];
state[156] = acadoVariables.od[index * 10 + 2];
state[157] = acadoVariables.od[index * 10 + 3];
state[158] = acadoVariables.od[index * 10 + 4];
state[159] = acadoVariables.od[index * 10 + 5];
state[160] = acadoVariables.od[index * 10 + 6];
state[161] = acadoVariables.od[index * 10 + 7];
state[162] = acadoVariables.od[index * 10 + 8];
state[163] = acadoVariables.od[index * 10 + 9];

acado_integrate(state, index == 0);

acadoVariables.x[index * 10 + 10] = state[0];
acadoVariables.x[index * 10 + 11] = state[1];
acadoVariables.x[index * 10 + 12] = state[2];
acadoVariables.x[index * 10 + 13] = state[3];
acadoVariables.x[index * 10 + 14] = state[4];
acadoVariables.x[index * 10 + 15] = state[5];
acadoVariables.x[index * 10 + 16] = state[6];
acadoVariables.x[index * 10 + 17] = state[7];
acadoVariables.x[index * 10 + 18] = state[8];
acadoVariables.x[index * 10 + 19] = state[9];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 10] = acadoVariables.x[index * 10 + 10];
acadoVariables.x[index * 10 + 1] = acadoVariables.x[index * 10 + 11];
acadoVariables.x[index * 10 + 2] = acadoVariables.x[index * 10 + 12];
acadoVariables.x[index * 10 + 3] = acadoVariables.x[index * 10 + 13];
acadoVariables.x[index * 10 + 4] = acadoVariables.x[index * 10 + 14];
acadoVariables.x[index * 10 + 5] = acadoVariables.x[index * 10 + 15];
acadoVariables.x[index * 10 + 6] = acadoVariables.x[index * 10 + 16];
acadoVariables.x[index * 10 + 7] = acadoVariables.x[index * 10 + 17];
acadoVariables.x[index * 10 + 8] = acadoVariables.x[index * 10 + 18];
acadoVariables.x[index * 10 + 9] = acadoVariables.x[index * 10 + 19];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[200] = xEnd[0];
acadoVariables.x[201] = xEnd[1];
acadoVariables.x[202] = xEnd[2];
acadoVariables.x[203] = xEnd[3];
acadoVariables.x[204] = xEnd[4];
acadoVariables.x[205] = xEnd[5];
acadoVariables.x[206] = xEnd[6];
acadoVariables.x[207] = xEnd[7];
acadoVariables.x[208] = xEnd[8];
acadoVariables.x[209] = xEnd[9];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[200];
state[1] = acadoVariables.x[201];
state[2] = acadoVariables.x[202];
state[3] = acadoVariables.x[203];
state[4] = acadoVariables.x[204];
state[5] = acadoVariables.x[205];
state[6] = acadoVariables.x[206];
state[7] = acadoVariables.x[207];
state[8] = acadoVariables.x[208];
state[9] = acadoVariables.x[209];
if (uEnd != 0)
{
state[150] = uEnd[0];
state[151] = uEnd[1];
state[152] = uEnd[2];
state[153] = uEnd[3];
}
else
{
state[150] = acadoVariables.u[76];
state[151] = acadoVariables.u[77];
state[152] = acadoVariables.u[78];
state[153] = acadoVariables.u[79];
}
state[154] = acadoVariables.od[200];
state[155] = acadoVariables.od[201];
state[156] = acadoVariables.od[202];
state[157] = acadoVariables.od[203];
state[158] = acadoVariables.od[204];
state[159] = acadoVariables.od[205];
state[160] = acadoVariables.od[206];
state[161] = acadoVariables.od[207];
state[162] = acadoVariables.od[208];
state[163] = acadoVariables.od[209];

acado_integrate(state, 1);

acadoVariables.x[200] = state[0];
acadoVariables.x[201] = state[1];
acadoVariables.x[202] = state[2];
acadoVariables.x[203] = state[3];
acadoVariables.x[204] = state[4];
acadoVariables.x[205] = state[5];
acadoVariables.x[206] = state[6];
acadoVariables.x[207] = state[7];
acadoVariables.x[208] = state[8];
acadoVariables.x[209] = state[9];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[76] = uEnd[0];
acadoVariables.u[77] = uEnd[1];
acadoVariables.u[78] = uEnd[2];
acadoVariables.u[79] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79];
kkt = fabs( kkt );
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index + 80];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 14 */
real_t tmpDy[ 14 ];

/** Row vector of size: 10 */
real_t tmpDyN[ 10 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 10];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 10 + 1];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 10 + 2];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 10 + 3];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 10 + 4];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 10 + 5];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 10 + 6];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 10 + 7];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 10 + 8];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 10 + 9];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 14] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 14];
acadoWorkspace.Dy[lRun1 * 14 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 14 + 1];
acadoWorkspace.Dy[lRun1 * 14 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 14 + 2];
acadoWorkspace.Dy[lRun1 * 14 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 14 + 3];
acadoWorkspace.Dy[lRun1 * 14 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 14 + 4];
acadoWorkspace.Dy[lRun1 * 14 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 14 + 5];
acadoWorkspace.Dy[lRun1 * 14 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 14 + 6];
acadoWorkspace.Dy[lRun1 * 14 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 14 + 7];
acadoWorkspace.Dy[lRun1 * 14 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 14 + 8];
acadoWorkspace.Dy[lRun1 * 14 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 14 + 9];
acadoWorkspace.Dy[lRun1 * 14 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 14 + 10];
acadoWorkspace.Dy[lRun1 * 14 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 14 + 11];
acadoWorkspace.Dy[lRun1 * 14 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 14 + 12];
acadoWorkspace.Dy[lRun1 * 14 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 14 + 13];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[200];
acadoWorkspace.objValueIn[1] = acadoVariables.x[201];
acadoWorkspace.objValueIn[2] = acadoVariables.x[202];
acadoWorkspace.objValueIn[3] = acadoVariables.x[203];
acadoWorkspace.objValueIn[4] = acadoVariables.x[204];
acadoWorkspace.objValueIn[5] = acadoVariables.x[205];
acadoWorkspace.objValueIn[6] = acadoVariables.x[206];
acadoWorkspace.objValueIn[7] = acadoVariables.x[207];
acadoWorkspace.objValueIn[8] = acadoVariables.x[208];
acadoWorkspace.objValueIn[9] = acadoVariables.x[209];
acadoWorkspace.objValueIn[10] = acadoVariables.od[200];
acadoWorkspace.objValueIn[11] = acadoVariables.od[201];
acadoWorkspace.objValueIn[12] = acadoVariables.od[202];
acadoWorkspace.objValueIn[13] = acadoVariables.od[203];
acadoWorkspace.objValueIn[14] = acadoVariables.od[204];
acadoWorkspace.objValueIn[15] = acadoVariables.od[205];
acadoWorkspace.objValueIn[16] = acadoVariables.od[206];
acadoWorkspace.objValueIn[17] = acadoVariables.od[207];
acadoWorkspace.objValueIn[18] = acadoVariables.od[208];
acadoWorkspace.objValueIn[19] = acadoVariables.od[209];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9] - acadoVariables.yN[9];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 14] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 28] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 42] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 56] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 70] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 84] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 98] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 112] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 126] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 140] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 154] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 168] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 182];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 1] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 15] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 29] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 43] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 57] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 71] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 85] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 99] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 113] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 127] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 141] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 155] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 169] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 183];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 2] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 16] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 30] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 44] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 58] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 72] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 86] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 100] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 114] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 128] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 142] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 156] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 170] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 184];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 3] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 17] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 31] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 45] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 59] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 73] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 87] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 101] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 115] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 129] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 143] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 157] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 171] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 185];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 4] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 18] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 32] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 46] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 60] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 74] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 88] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 102] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 116] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 130] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 144] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 158] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 172] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 186];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 5] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 19] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 33] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 47] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 61] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 75] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 89] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 103] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 117] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 131] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 145] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 159] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 173] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 187];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 6] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 20] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 34] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 48] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 62] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 76] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 90] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 104] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 118] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 132] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 146] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 160] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 174] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 188];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 7] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 21] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 35] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 49] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 63] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 77] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 91] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 105] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 119] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 133] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 147] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 161] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 175] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 189];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 8] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 22] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 36] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 50] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 64] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 78] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 92] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 106] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 120] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 134] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 148] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 162] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 176] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 190];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 9] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 23] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 37] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 51] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 65] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 79] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 93] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 107] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 121] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 135] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 149] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 163] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 177] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 191];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 10] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 24] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 38] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 52] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 66] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 80] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 94] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 108] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 122] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 136] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 150] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 164] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 178] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 192];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 11] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 25] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 39] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 53] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 67] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 81] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 95] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 109] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 123] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 137] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 151] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 165] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 179] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 193];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 12] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 26] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 40] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 54] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 68] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 82] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 96] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 110] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 124] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 138] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 152] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 166] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 180] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 194];
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 14]*acadoVariables.W[lRun1 * 196 + 13] + acadoWorkspace.Dy[lRun1 * 14 + 1]*acadoVariables.W[lRun1 * 196 + 27] + acadoWorkspace.Dy[lRun1 * 14 + 2]*acadoVariables.W[lRun1 * 196 + 41] + acadoWorkspace.Dy[lRun1 * 14 + 3]*acadoVariables.W[lRun1 * 196 + 55] + acadoWorkspace.Dy[lRun1 * 14 + 4]*acadoVariables.W[lRun1 * 196 + 69] + acadoWorkspace.Dy[lRun1 * 14 + 5]*acadoVariables.W[lRun1 * 196 + 83] + acadoWorkspace.Dy[lRun1 * 14 + 6]*acadoVariables.W[lRun1 * 196 + 97] + acadoWorkspace.Dy[lRun1 * 14 + 7]*acadoVariables.W[lRun1 * 196 + 111] + acadoWorkspace.Dy[lRun1 * 14 + 8]*acadoVariables.W[lRun1 * 196 + 125] + acadoWorkspace.Dy[lRun1 * 14 + 9]*acadoVariables.W[lRun1 * 196 + 139] + acadoWorkspace.Dy[lRun1 * 14 + 10]*acadoVariables.W[lRun1 * 196 + 153] + acadoWorkspace.Dy[lRun1 * 14 + 11]*acadoVariables.W[lRun1 * 196 + 167] + acadoWorkspace.Dy[lRun1 * 14 + 12]*acadoVariables.W[lRun1 * 196 + 181] + acadoWorkspace.Dy[lRun1 * 14 + 13]*acadoVariables.W[lRun1 * 196 + 195];
objVal += + acadoWorkspace.Dy[lRun1 * 14]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 14 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 14 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 14 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 14 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 14 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 14 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 14 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 14 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 14 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 14 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 14 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 14 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 14 + 13]*tmpDy[13];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[11];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[22];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[33];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[44];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[55];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[66];
tmpDyN[7] = + acadoWorkspace.DyN[7]*acadoVariables.WN[77];
tmpDyN[8] = + acadoWorkspace.DyN[8]*acadoVariables.WN[88];
tmpDyN[9] = + acadoWorkspace.DyN[9]*acadoVariables.WN[99];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8] + acadoWorkspace.DyN[9]*tmpDyN[9];

objVal *= 0.5;
return objVal;
}

