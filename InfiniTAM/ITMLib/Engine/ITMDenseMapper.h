// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "ITMSceneReconstructionEngine.h"
#include "ITMVisualisationEngine.h"
#include "ITMSwappingEngine.h"
#include "ITMVoxelBlockOpEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		template<class TVoxel, class TIndex>
		class ITMDenseMapper
		{
		private:
			ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine;
			ITMSwappingEngine<TVoxel,TIndex> *swappingEngine;
			ITMVoxelBlockOpEngine<TVoxel,TIndex> *voxelBlockOpEngine;

		public:
			/// Process a single frame
			void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState_live);

			/// Update the visible list (this can be called to update the visible list when fusion is turned off)
			void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState);

			/** \brief Constructor
			    Ommitting a separate image size for the depth images
			    will assume same resolution as for the RGB images.
			*/
			explicit ITMDenseMapper(const ITMLibSettings *settings);
			~ITMDenseMapper();
		};
	}
}

