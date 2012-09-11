#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include "Core/inc/Core.h"
#include "Core/inc/CommandlineParser.h"
#include "Core/inc/Exception.h"
#include "Core/inc/io/File.h"
#include "Core/inc/io/FileSystem.h"
#include "Core/inc/Log.h"
#include "Core/inc/nString.h"
#include "Core/inc/NumericCast.h"
#include "Core/inc/StringFormat.h"
#include "Core/inc/Iterator3D.h"
#include "Core/inc/math/3dUtility.h"
#include "Core/inc/math/VectorToString.h"
#include "niven.Core.MemoryLayout3D.h"
#include "niven.Core.Math.ArrayFunctions.h"
#include "niven.Core.Math.VectorFunctions.h"

#include "Volume/inc/MarchingCubes.h"
#include "niven.Volume.FileBlockStorage.h"
#include "niven.Core.IO.Path.h"

#include "Core/inc/Timer.h"
#include "niven.Core.PointerHelper.h"
#include "niven.Core.Generic.ScopedArray.h"

#include <iostream>

using namespace niven;

class ShardFileParser {
public:
	NIV_DEFINE_CLASS_SHARED_PTR(ShardFileParser);
public:
	ShardFileParser(const String& volumeName, float isoValue);
	float GetDensity(int x, int y, int z) const;
	int getVolumeHeight() const;
	int getVolumeWidth() const;
	int getVolumeDepth() const;
	int getBorderSize() const;

private:
	void creatOwnVolumeFile(const String& volumeName);

private:
	String volumeName_;
	float isoValue_;
	Volume::FileBlockStorage volume_;
	MemoryLayout3D layout_;
	std::vector<float> density_;
	int borderSize_;
};