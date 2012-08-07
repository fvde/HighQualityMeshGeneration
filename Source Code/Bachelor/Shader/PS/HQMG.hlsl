/**
* DebugShader
*/

#include "vertex-factory.hlsl"

float4 PS_main (VertexToFragment vtf) : SV_Target 
{
	return float4(dot(ctf.color.xyz, vtf.normal.xyz), 1.0f);
}