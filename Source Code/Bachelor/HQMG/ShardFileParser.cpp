#include "ShardFileParser.h"


ShardFileParser::ShardFileParser(const String& volumeName, float isoValue)
	: volumeName_(volumeName),
	isoValue_(isoValue){

		// Open the shardfile
		creatOwnVolumeFile();
		volume_.Open("smallSphere.nvf", true);
		//volume_.Open (volumeName_, true);
		const String layerName = "Density";

		if(volume_.ContainsLayer(layerName)){

			const auto ids = volume_.GetBlockIds(layerName);
			const auto layerDescriptor = volume_.GetLayerDescriptor(layerName);
			borderSize_ = layerDescriptor.borderSize;
			layout_ = MemoryLayout3D(layerDescriptor.blockResolution + borderSize_ * 2);
			density_ = std::vector<float>(layout_.GetStorageSize());

			for (auto it = ids.begin (), end = ids.end (); it != end; ++it) 
			{
				volume_.GetBlock(layerName, *it, density_);
			}
		} else {
			throw Exception("No density layer in the volume file!", "...");
		}
}

float ShardFileParser::GetDensity(int x, int y, int z) const {
	if(x < getBorderSize() || 
		y < getBorderSize() || 
		z < getBorderSize() || 
		x >= getVolumeWidth() - getBorderSize() || 
		y >= getVolumeHeight() - getBorderSize() || 
		z >= getVolumeDepth() - getBorderSize()){
			return 0;
	} else {
		return density_ [layout_ (x, y, z)];
	}
}

int ShardFileParser::getVolumeHeight() const {
	return layout_.GetHeight();
}

int ShardFileParser::getVolumeWidth() const {
	return layout_.GetWidth();
}

int ShardFileParser::getVolumeDepth() const {
	return layout_.GetDepth();
}

int ShardFileParser::getBorderSize() const {
	return borderSize_;
}


void ShardFileParser::creatOwnVolumeFile(){
int volumeSize = 4;
int volumeBorderSize = 0;
const String layerName = "Density";

auto volume = Volume::FileBlockStorage ();
volume.Create ("smallSphere.nvf");

Volume::FileBlockStorage::Metadata fileMetadata;
fileMetadata.blockSize = volumeSize;

volume.SetMetadata (fileMetadata);

Volume::IBlockStorage::LayerDescriptor layerDesc;
layerDesc.blockResolution = volumeSize;
layerDesc.borderSize = volumeBorderSize;
layerDesc.dataType = DataType::Float;

// Create the layer
volume.AddLayer(layerName, layerDesc);

// Add blocks

ScopedArray<float> volumeData (Math::Cube (volumeSize + 2 * volumeBorderSize));
MemoryLayout3D l = MemoryLayout3D(volumeSize + 2 * volumeBorderSize);

const auto size = volumeData.GetCount ();
for(int x = 0; x < size; ++x){
volumeData[x] = 0.0f;
}

Iterator3D it (volumeSize, volumeSize, volumeSize), end;
const Vector3f center (volumeSize / 2.0f, volumeSize / 2.0f, volumeSize / 2.0f);

while (it != end){
const float distance = Length (center - it.ToVector ().Cast<float> ()) / (volumeSize / 2.0f);
volumeData[l(it.X() + volumeBorderSize, it.Y() + volumeBorderSize, it.Z() + volumeBorderSize)] = 1 - Math::Clamp (distance, 0.0f, 1.0f);
++it;
}

const Volume::IBlockStorage::Id id = { Vector3i (0, 0, 0), 0 };
volume.AddBlock(layerName, id, ArrayRef<void>(volumeData.Get (), volumeData.GetSize ()));
}


