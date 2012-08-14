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
#include "niven.Engine.DebugRenderUtility.h"

#include "Core/inc/Iterator3D.h"
#include "Core/inc/math/3dUtility.h"
#include "Core/inc/math/VectorToString.h"
#include "niven.Core.MemoryLayout3D.h"

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
	CMD_ENTER,
	CMD_0,
	CMD_1,
	CMD_2,
	CMD_3,
	};

	enum DrawMode {
		Mesh = 1,
		Intersections = 2,
		Wireframe = 3,
		All = 0
	};

private:
	String volumeName_;
	float isoValue_;
	int visibleTriangleCounter_;
	Render::DebugRenderUtility dru_;
	MeshExtractor me_;
	std::vector<Render::VertexFormats::PositionNormalColorUv> vb_;
	std::vector<Vector3f> vertices_;
	std::vector<Vector3f> normals_;
	std::vector<Vector4f> colors_;
	std::vector<uint32> indices_;

	std::tr1::shared_ptr<niven::Render::IIndexBuffer> indexBuffer_;
	std::tr1::shared_ptr<niven::Render::IVertexBuffer> vertexBuffer_;

	std::vector<Color3f> debugColors_;
	DrawMode drawMode_;

	int stepStart_;
	int stepIntervall_;

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

		commandMapper_.RegisterCommand (CMD_ENTER, KeyBinding (KeyCodes::Key_Return,
			KeyBindingActivator::Pressed));

		commandMapper_.RegisterCommand (CMD_0, KeyBinding (KeyCodes::Key_0,
			KeyBindingActivator::Pressed));

		commandMapper_.RegisterCommand (CMD_1, KeyBinding (KeyCodes::Key_1,
			KeyBindingActivator::Pressed));

		commandMapper_.RegisterCommand (CMD_2, KeyBinding (KeyCodes::Key_2,
			KeyBindingActivator::Pressed));

		commandMapper_.RegisterCommand (CMD_3, KeyBinding (KeyCodes::Key_3,
			KeyBindingActivator::Pressed));

		camera_->GetFrustum().SetPerspectiveProjection(
			Degree (75.0f),
			renderWindow_->GetAspectRatio(),
			0.01f, 1024.0f);

		camera_->SetViewLookAt(Vector3f(-2.5f, -2.0f, -0.5f),
			Vector3f(16, 16, 16),
			Vector3f(0, 1, 0));

		camera_->SetMoveSpeedMultiplier(5.0f);

		srand(time(NULL));
		visibleTriangleCounter_ = 1;
		drawMode_ = DrawMode::All;
		//stepStart_ = 316;
		stepStart_ = 1;
		stepIntervall_ = 1;

		effectManager_.Initialize(renderSystem_.get(), &effectLoader_);
		
		effect_ = effectManager_.GetEffect ("HQMGDefault", 
			Render::VertexFormats::PositionNormalColorUv::GetId ());
		
		vertexLayout_ = renderSystem_->CreateVertexLayout (
			Render::VertexFormats::PositionNormalColorUv::GetVertexLayout (),
			effect_->GetVertexShaderProgram ());


		ShardFileParser::Ptr sfp = cpp0x::make_shared<ShardFileParser> (volumeName_, isoValue_);

		Log::Debug("High Quality Mesh Generation", "Starting triangulation...");
		me_ = MeshExtractor(sfp, isoValue_, 0.3f, 1.2f);
		me_.Setup();
		Log::Debug("High Quality Mesh Generation", "Ready for triangulation!");


		indexBuffer_ = renderSystem_->Wrap (renderSystem_->CreateIndexBuffer (
		IndexBufferFormat::UInt_32,
		numeric_cast<int> (4242424),
		Render::ResourceUsage::Dynamic));

		vertexBuffer_ = renderSystem_->Wrap (renderSystem_->CreateVertexBuffer (
		sizeof (Render::VertexFormats::PositionNormalColorUv),
		numeric_cast<int> (4242424),
		Render::ResourceUsage::Dynamic));

		mesh_.vertexBuffer = vertexBuffer_;
		mesh_.indexBuffer = indexBuffer_;

		debugColors_.resize(20);
		// 0-8 fronts.
		debugColors_[0] = Color3f(0, 1.0f, 0);
		debugColors_[1] = Color3f(0, 0, 1.0f);
		debugColors_[2] = Color3f(0, 1.0f, 1.0f);
		debugColors_[3] = Color3f(0, 0.25f, 0.25f);
		debugColors_[4] = Color3f(0, 0.25f, 0);
		debugColors_[5] = Color3f(0, 0, 0.25f);
		debugColors_[6] = Color3f(0, 0.75f, 0);
		debugColors_[7] = Color3f(0, 0, 0.75f);
		debugColors_[8] = Color3f(0, 0.25f, 0.75f);
		// 9: Last Attempt triangle pre merge/split 10: Last Attempt triangle after split/merge
		debugColors_[9] = Color3f(1.0f, 0, 0);
		debugColors_[10] = Color3f(1.0f, 1.0f, 0);
		// 11: Edges in range. 12: intersection edges, 13: intersection
		debugColors_[11] = Color3f(1.0f, 0, 0);
		debugColors_[12] = Color3f(0, 1.0f, 0);
		debugColors_[13] = Color3f(0, 0, 1.0f);
		debugColors_[14] = Color3f(1.0f, 1.0f, 0);

		if (stepStart_> 0){
			ExecuteNextStep(stepStart_);
		}
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
		case CMD_ENTER:{
			ExecuteNextStep(stepIntervall_);
			break;
			}
		case CMD_0:{
			drawMode_ = DrawMode::All;
			dru_.Clear();
			AddMeshDebugInformation();
			AddIntersectionDebugInformation();
			//AddWireFrame();
			break;
			}
		case CMD_1:{
			drawMode_ = DrawMode::Mesh;
			dru_.Clear();
			AddMeshDebugInformation();
			break;
			}
		case CMD_2:{
			drawMode_ = DrawMode::Intersections;
			dru_.Clear();
			AddIntersectionDebugInformation();
			break;
			}
		case CMD_3:{
			drawMode_ = DrawMode::Wireframe;
			dru_.Clear();
			AddWireFrame();
			break;
			}
		}
	}

	void ExecuteNextStep(int stepCount){

		int previousSize = vertices_.size();

		me_.TriangulateShardFile(vertices_, normals_, colors_, indices_, stepCount);

		vb_.resize(vertices_.size ());

		visibleTriangleCounter_ = vertices_.size () / 3;

		for (int i = previousSize; i < static_cast<int>(vertices_.size ()); ++i)
			{
				vb_[i] = (Render::VertexFormats::PositionNormalColorUv::Create (
					vertices_ [i], normals_[i], colors_[i], Vector2f(0,0)));
			}
		
		
		void* d;
		vertexBuffer_->Map (renderContext_, &d, Render::ResourceAccessMode::Cpu_Write,
			Render::ResourceMapFlags::Discard);
		::memcpy (d, vb_.data (), vb_.size () * sizeof (Render::VertexFormats::PositionNormalColorUv));
		vertexBuffer_->Unmap (renderContext_);

		void* t;
		indexBuffer_->Map (renderContext_, &t, Render::ResourceAccessMode::Cpu_Write,
			Render::ResourceMapFlags::Discard);
		::memcpy (t, indices_.data (), indices_.size () * sizeof (IndexBufferFormat::UInt_32));
		indexBuffer_->Unmap (renderContext_);

		mesh_.vertexCount = numeric_cast<int> (vertices_.size ());
		mesh_.indexCount = numeric_cast<int> (indices_.size ());

		// Debugging
		dru_.Clear();
		switch (drawMode_) {
			case DrawMode::All:{
				AddMeshDebugInformation();
				AddIntersectionDebugInformation();
				//AddWireFrame();
				break;
				}
			case DrawMode::Mesh:{
				AddMeshDebugInformation();
				break;
				}
			case DrawMode::Intersections:{
				AddIntersectionDebugInformation();
				break;
				}
			case DrawMode::Wireframe:{
				AddWireFrame();
				break;
				}
		}

		for (auto it = me_.AdditionalDebugInfo.begin(); it != me_.AdditionalDebugInfo.end(); it++){
			dru_.AddCross((*it), Color3f(1.0f, 0.5f, 0.5f), 0.05);
		}
		me_.AdditionalDebugInfo.clear();
	}

	void AddIntersectionDebugInformation() {
		for (auto it = me_.LastEdgesInRange.begin(); it != me_.LastEdgesInRange.end(); it++){
			dru_.AddLine(it->v1, it->v2, debugColors_[11]);
		}

		for (auto it = me_.LastIntersectionEdges.begin(); it != me_.LastIntersectionEdges.end(); it++){
			dru_.AddLine(it->v1, it->v2, debugColors_[12]);
		}

		for (auto it = me_.Intersections.begin(); it != me_.Intersections.end(); it++){
			dru_.AddCross((*it), debugColors_[13], 0.02f);
		}

		dru_.AddLine(me_.TransformedPostOrigin, me_.TransformedPostNeighbor, debugColors_[14]);
		dru_.AddLine(me_.TransformedPostNeighbor, me_.TransformedPostAttempt, debugColors_[14]);
		dru_.AddLine(me_.TransformedPostAttempt, me_.TransformedPostOrigin, debugColors_[14]);
	}

	void AddWireFrame() {
		for (auto it = 0; it < vertices_.size(); it += 3){
			dru_.AddLine(vertices_[it], vertices_[it + 1], Color3f(0.0f, 1.0f, 1.0f));
			dru_.AddLine(vertices_[it + 1], vertices_[it + 2], Color3f(0.0f, 1.0f, 1.0f));
			dru_.AddLine(vertices_[it + 2], vertices_[it], Color3f(0.0f, 1.0f, 1.0f));
		}
	}

	void AddMeshDebugInformation() {

		// Draw fronts
		for (int outer = 0; outer < me_.DebugFront.size(); outer++){
			for (int inner = 0; inner < me_.DebugFront[outer].size() - 1; inner++){
				dru_.AddLine(me_.DebugFront[outer][inner], me_.DebugFront[outer][inner + 1], debugColors_[Min(8, outer)]);
			}
			dru_.AddLine(me_.DebugFront[outer][me_.DebugFront[outer].size() - 1], me_.DebugFront[outer][0], debugColors_[Min(8, outer)]);
		}

		// Draw last triangle pre and after operations
		// Pre:
		dru_.AddLine(me_.PreOrigin, me_.PreNeighbor, debugColors_[9]);
		dru_.AddLine(me_.PreNeighbor, me_.PreAttempt, debugColors_[9]);
		dru_.AddLine(me_.PreAttempt, me_.PreOrigin, debugColors_[9]);

		dru_.AddCross(me_.PreOrigin, debugColors_[9], 0.02f);
		dru_.AddCross(me_.PreAttempt, debugColors_[9], 0.02f);
		dru_.AddCross(me_.PreNeighbor, debugColors_[9], 0.02f);

		// Post:
		dru_.AddLine(me_.PostOrigin, me_.PostAttempt, debugColors_[10]);
		dru_.AddLine(me_.PostNeighbor, me_.PostAttempt, debugColors_[10]);
		dru_.AddLine(me_.PostNeighbor, me_.PostOrigin, debugColors_[10]);

		dru_.AddCross(me_.PostOrigin, debugColors_[10], 0.02f);
		dru_.AddCross(me_.PostAttempt, debugColors_[10], 0.02f);
		dru_.AddCross(me_.PostNeighbor, debugColors_[10], 0.02f);
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
		renderSystem_->DebugDrawLines(dru_.GetLineSegments());
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
