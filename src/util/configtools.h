/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONFIGTOOLS_H_
#define CONFIGTOOLS_H_

#include <dvo_core/dense_tracking.h>
#include <dvo/CameraDenseTrackerConfig.h>


void updateConfigFromDynamicReconfigure(const dvo::CameraDenseTrackerConfig& config, DenseTracker::Config& tracker_cfg)
{
  ScaleEstimators::enum_t scale_estimator;

  switch(config.scale_estimator)
  {
    case dvo::CameraDenseTracker_NormalDistributionScaleEstimator:
      scale_estimator = ScaleEstimators::NormalDistribution;
      break;
    case dvo::CameraDenseTracker_TDistributionScaleEstimator:
      scale_estimator = ScaleEstimators::TDistribution;
      break;
    case dvo::CameraDenseTracker_MADScaleEstimator:
      scale_estimator = ScaleEstimators::MAD;
      break;
    default:
      assert(false && "unknown scale estimator");
      break;
  }

  InfluenceFunctions::enum_t influence_function;

  switch(config.influence_function)
  {
    case dvo::CameraDenseTracker_TukeyInfluenceFunction:
      influence_function = InfluenceFunctions::Tukey;
      break;
    case dvo::CameraDenseTracker_TDistributionInfluenceFunction:
      influence_function = InfluenceFunctions::TDistribution;
      break;
    case dvo::CameraDenseTracker_HuberInfluenceFunction:
      influence_function = InfluenceFunctions::Huber;
      break;
    default:
      assert(false && "unknown influence function");
      break;
  }

  tracker_cfg.FirstLevel = config.coarsest_level;
  tracker_cfg.LastLevel = config.finest_level;
  tracker_cfg.MaxIterationsPerLevel = config.max_iterations;
  tracker_cfg.Precision = config.precision;
  tracker_cfg.UseInitialEstimate = config.use_initial_estimate;
  tracker_cfg.UseWeighting = config.use_weighting;
  tracker_cfg.ScaleEstimatorType = scale_estimator;
  tracker_cfg.ScaleEstimatorParam = config.scale_estimator_param;
  tracker_cfg.InfluenceFuntionType = influence_function;
  tracker_cfg.InfluenceFunctionParam = config.influence_function_param;
  tracker_cfg.Lambda = config.lambda;
  tracker_cfg.Mu = config.mu;
}


#endif /* CONFIGTOOLS_H_ */
