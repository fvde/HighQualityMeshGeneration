/**
* @author  Matthaeus G. "Anteru" Chajdas
*
* License: NDL
*/

#include "niven.Core.Core.h"
#include "Core/inc/CommandlineParser.h"
#include "Core/inc/Exception.h"
#include "Core/inc/io/File.h"
#include "Core/inc/io/FileSystem.h"
#include "Core/inc/Log.h"
#include "Core/inc/nString.h"
#include "Core/inc/NumericCast.h"
#include "Core/inc/StringFormat.h"

#include "niven.Engine.BaseApplication3D.h"
#include "niven.Render.DrawCommand.h"
#include "niven.Render.Effect.h"
#include "niven.Render.EffectLoader.h"
#include "niven.Render.EffectManager.h"
#include "niven.Render.IndexBuffer.h"
#include "niven.Render.RenderBuffer.h"
#include "niven.Render.RenderContext.h"
#include "niven.Render.RenderSystem.h"
#include "niven.Render.RenderTargetDescriptor.h"
#include "niven.Render.RenderTargetTexture3D.h"
#include "niven.Render.Texture3D.h"
#include "niven.Render.VertexBuffer.h"
#include "niven.Render.VertexFormats.All.h"
#include "niven.Render.VertexLayout.h"

#include "Core/inc/Iterator3D.h"
#include "Core/inc/math/3dUtility.h"
#include "Core/inc/math/VectorToString.h"
#include "Core/inc/MemoryLayout.h"

#include "Volume/inc/MarchingCubes.h"

#include "Core/inc/Timer.h"
#include "ShardFileParser.h"
#include "MeshExtractor.h"
#include "stdio.h"

#include <iostream>

using namespace niven;

class HQMG2DApplication : public BaseApplication3D {
	NIV_DEFINE_CLASS(HQMG2DApplication, BaseApplication3D)

	enum {
	CMD_UP = CMD_FirstUser,
	CMD_DOWN,
	CMD_LEFT,
	CMD_RIGHT,
	};

private:
	String volumeName_;
	float isoValue_;
	int visibleTriangleCounter_;


public:
	HQMG2DApplication(const String& volumeName, float isoValue)
		: volumeName_(volumeName),
		isoValue_(isoValue)
	{
	}

	// Default constructor is needed for NIV_DEFINE_CLASS
	HQMG2DApplication() {
	}

private:
	void InitImpl() {
		Super::InitImpl();

		commandMapper_.RegisterCommand (CMD_UP, KeyBinding (KeyCodes::Key_Cursor_Up,
			KeyBindingActivator::Pressed));

		commandMapper_.RegisterCommand (CMD_DOWN, KeyBinding (KeyCodes::Key_Cursor_Down,
			KeyBindingActivator::Pressed));

		commandMapper_.RegisterCommand (CMD_LEFT, KeyBinding (KeyCodes::Key_Cursor_Left,
			KeyBindingActivator::Pressed));

		commandMapper_.RegisterCommand (CMD_RIGHT, KeyBinding (KeyCodes::Key_Cursor_Right,
			KeyBindingActivator::Pressed));

		camera_->GetFrustum().SetPerspectiveProjection(
			Degree (75.0f),
			renderWindow_->GetAspectRatio(),
			1.0f, 1024.0f);

		camera_->SetViewLookAt(Vector3f(-2.5f, -2.0f, -0.5f),
			Vector3f(16, 16, 16),
			Vector3f(0, 1, 0));

		camera_->SetMoveSpeedMultiplier(10.0f);

		effectManager_.Initialize(renderSystem_.get(), &effectLoader_);
		
		effect_ = effectManager_.GetEffect ("HQMGDefault", 
			Render::VertexFormats::PositionNormalColorUv::GetId ());
		
		vertexLayout_ = renderSystem_->CreateVertexLayout (
			Render::VertexFormats::PositionNormalColorUv::GetVertexLayoutElementCount (),
			Render::VertexFormats::PositionNormalColorUv::GetVertexLayout (),
			effect_->GetVertexShaderProgram ());


		ShardFileParser::Ptr sfp = cpp0x::make_shared<ShardFileParser> (volumeName_, isoValue_);

		Log::Debug("High Quality Mesh Generation", "Starting triangulation...");
		MeshExtractor me(sfp, isoValue_, 0.3f, 1.2f);

		std::vector<Vector3f> vertices;
		std::vector<Vector3f> normals;
		std::vector<Vector4f> colors; // not used atm
		std::vector<uint32> indices;


		me.TriangulateShardFile(vertices, normals, colors, indices);
		visibleTriangleCounter_ = 1;

		if (indices.empty () || vertices.empty() || normals.empty() || colors.empty())
				{
					throw Exception("No vertices or indices or normals or colors where created!", "Check the implementation of the mesh extraction algorithm for more information.");
				}

		Log::Debug("High Quality Mesh Generation", "Completed triangulation!");

		auto indexBuffer = renderSystem_->Wrap (renderSystem_->CreateIndexBuffer (
				IndexBufferFormat::UInt_32,
				numeric_cast<int> (indices.size ()),
				Render::ResourceUsage::Static,
				&indices [0]));

		std::vector<Render::VertexFormats::PositionNormalColorUv> vb (vertices.size ());
		
		for (int i = 0; i < static_cast<int>(vertices.size ()); ++i)
			{
				vb[i] = (Render::VertexFormats::PositionNormalColorUv::Create (
					vertices [i], normals[i], colors[i], Vector2f(0,0)));
			}
		

		auto vertexBuffer = renderSystem_->Wrap (renderSystem_->CreateVertexBuffer (
				sizeof (Render::VertexFormats::PositionNormalColorUv),
				numeric_cast<int> (vb.size ()),
				Render::ResourceUsage::Static,
				&vb[0]));
				

		mesh_.vertexCount = numeric_cast<int> (vertices.size ());
		mesh_.indexCount = numeric_cast<int> (indices.size ());
		mesh_.vertexBuffer = vertexBuffer;
		mesh_.indexBuffer = indexBuffer;
	}
	void HandleCommand(const CommandID id, const InputContext *context)
	{
	Super::HandleCommand (id, context);	
	switch (id) {
		case CMD_UP:{
			visibleTriangleCounter_++;
			break;
					}
		case CMD_DOWN:{
			if( visibleTriangleCounter_ > 1){
				visibleTriangleCounter_--;
			}
			break;
			}
		case CMD_RIGHT:{
			visibleTriangleCounter_ += 100;
			break;
			}
		case CMD_LEFT:{
			if( visibleTriangleCounter_ > 101){
				visibleTriangleCounter_ -= 100;;
			} else {
				visibleTriangleCounter_ = 1;
			}
			break;
			}
		}
	}

	void ShutdownImpl() 
	{
		renderSystem_->Release (vertexLayout_);

		effectManager_.Shutdown ();

		Super::ShutdownImpl();
	}

	void DrawImpl() 
	{
		
		renderContext_->SetTransformation (RenderTransformation::World,
			Matrix4f::CreateIdentity ());

		DrawIndexedCommand dic;
		dic.SetIndexBuffer(mesh_.indexBuffer);
		dic.SetVertexBuffer(mesh_.vertexBuffer.get());
		dic.SetVertexLayout(vertexLayout_);
		dic.type = PrimitiveType::Triangle_List;
		dic.indexCount = visibleTriangleCounter_ * 3;
		dic.vertexCount = visibleTriangleCounter_ * 3;

		effect_->Bind (renderContext_);
		renderContext_->Draw(dic);
		effect_->Unbind (renderContext_);
	}

	String GetMainWindowName () const
	{
		return "High Quality Mesh Extraction";
	}
	
private:
	Render::EffectManager			effectManager_;
	Render::EffectLoader			effectLoader_;

	Render::Effect*					effect_;
	IVertexLayout*					vertexLayout_;

	struct TriangulatedMesh
	{
		int							vertexCount,
									indexCount;
		IVertexBuffer::Ptr			vertexBuffer;
		IIndexBuffer::Ptr			indexBuffer;
	};

	TriangulatedMesh				mesh_;
};

NIV_IMPLEMENT_CLASS(HQMG2DApplication, TypeInfo::Default, AppMarchingCubes)

	int main(int argc, char* argv[]) {
		CoreLifeTimeHelper clth;
		try {
			CommandlineParser clp;
			clp.Add (CommandlineOption ("volume", VariantType::String))
				.Add (CommandlineOption ("iso-value", Variant (0.5f)));

			const auto options = clp.Parse(argc, argv);

			if (!options.IsSet ("volume")) {
				Log::Error ("HQMG2DApplication", "No volume specified. Use -volume <name> "
					"to set the volume");
				return 1;
			}


			HQMG2DApplication application (options.Get<String> ("volume"),
				options.Get<float> ("iso-value"));

			application.Init();
			application.Run();
			application.Shutdown();
		} catch (Exception& e) {
			std::cerr << e.what () << std::endl;
			std::cerr << e.where () << std::endl;
		} catch (std::exception& e) {
			std::cerr << e.what () << std::endl;
		}

		return 0;
}
