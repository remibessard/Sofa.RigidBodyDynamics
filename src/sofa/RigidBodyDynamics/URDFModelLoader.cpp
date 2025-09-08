/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
 *                                                                             *
 * This program is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 2.1 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: The SOFA Team and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact@sofa-framework.org                             *
 ******************************************************************************/
#include <sofa/core/ObjectFactory.h>
#include <sofa/RigidBodyDynamics/URDFModelLoader.h>

#include <sofa/RigidBodyDynamics/Conversions.h>
#include <sofa/RigidBodyDynamics/GeometryConversions.h>
#include <sofa/RigidBodyDynamics/KinematicChainMapping.h>

#include <sofa/component/io/mesh/MeshOBJLoader.h>
#include <sofa/component/io/mesh/MeshVTKLoader.h>
#include <filesystem>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/mass/UniformMass.h>
//#include <sofa/component/collision/geometry/TriangleModel.h>
//#include <sofa/component/collision/geometry/LineModel.h>
//#include <sofa/component/collision/geometry/PointModel.h>
#include <sofa/component/constraint/lagrangian/model/FixedLagrangianConstraint.h>
#include <sofa/component/mapping/nonlinear/RigidMapping.h>
#include <sofa/gl/component/rendering3d/OglModel.h>

#include <SoftRobots/component/constraint/JointConstraint.h>


using namespace sofa::defaulttype;

using MechanicalObjectVec1 = sofa::component::statecontainer::MechanicalObject<Vec1Types>;
using MechanicalObjectRigid3 = sofa::component::statecontainer::MechanicalObject<Rigid3Types>;

namespace sofa::rigidbodydynamics
{

    URDFModelLoader::URDFModelLoader() : sofa::core::loader::SceneLoader(),
        d_modelDirectory(initData(&d_modelDirectory, "modelDirectory", "Directory containing robot models")),
        d_useFreeFlyerRootJoint(initData(&d_useFreeFlyerRootJoint, false, "useFreeFlyerRootJoint", "True if root joint is a Free Flyer joint, false if none")),
        d_addCollision(initData(&d_addCollision, true, "addCollision", "True if collision detection must be enabled for the robot (self-collision and other objects)")),
        d_addJointsActuators(initData(&d_addJointsActuators, false, "addJointsActuators", "True if SoftRobots.Inverse actuators objects must be set for each robot joint")),
        d_fixUniverse(initData(&d_fixUniverse, "fixUniverse", "if true, adds a FixedConstraint to universe Frame")),
        d_qRest(initData(&d_qRest, "qRest", "Rest configuration values of robot DoFs")),
        d_qInit(initData(&d_qInit, "qInit", "Initial configuration values of robot DoFs"))
  {
  }

  void URDFModelLoader::setModelDirectory(const std::string &f)
  {
    d_modelDirectory.setValue(f);
  }

  const std::string &URDFModelLoader::getModelDirectory() const
  {
    return d_modelDirectory.getValue();
  }

  void URDFModelLoader::setUseFreeFlyerRootJoint(bool useFreeFlyerRootJoint)
  {
    d_useFreeFlyerRootJoint.setValue(useFreeFlyerRootJoint);
  }

  bool URDFModelLoader::getUseFreeFlyerRootJoint() const
  {
    return d_useFreeFlyerRootJoint.getValue();
  }

  void URDFModelLoader::init()
  {
    // no op
  }

  bool URDFModelLoader::load()
  {


    const std::string &urdfFilename = getFilename();
    const std::string &modelDir = d_modelDirectory.getValue();
    const bool useFreeFlyerRootJoint = d_useFreeFlyerRootJoint.getValue();

    msg_info() << "Loading robot from URDF file: " << urdfFilename;
    msg_info() << "Model directory: " << modelDir;
    std::shared_ptr<pinocchio::Model> model;
    std::shared_ptr<pinocchio::GeometryModel> collisionModel;
    std::shared_ptr<pinocchio::GeometryModel> visualModel;
    std::vector<pinocchio::FrameIndex> body_center_of_mass_frames;
    std::vector<pinocchio::FrameIndex> extraFrames;

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node

    // clear robot scene tree
    {
      const auto robotNode = context->getChild("Articulated-chain");
      if (robotNode)
      {
        context->removeChild(robotNode);
        msg_info() << "Robot node was already present in robot scene tree. Removing it...";
      }
      const auto rootJointNode = context->getChild("RootJoint");
      if (rootJointNode)
      {
        context->removeChild(rootJointNode);
        msg_info() << "RootJoint node was already present in robot scene tree. Removing it...";
      }
      const auto modelNode = context->getChild("Model");
      if (modelNode)
      {
        context->removeChild(modelNode);
        msg_info() << "Model node was already present in robot scene tree. Removing it...";
      }
    }

    // load URDF model
    try
    {
      model = std::make_shared<pinocchio::Model>();
      if(useFreeFlyerRootJoint)
      {
        pinocchio::urdf::buildModel(urdfFilename, pinocchio::JointModelFreeFlyer(), *model);
        msg_info() << "Built robot model (with Free Flyer root joint) from URDF file: " << urdfFilename;
      }
      else
      {
        pinocchio::urdf::buildModel(urdfFilename, *model);
        msg_info() << "Built robot model (with fixed root joint) from URDF file: " << urdfFilename;
      }

      if(!model->check())
      {
        msg_warning() << "Model failed checking. URDF model does not comply with pinocchio specifications";
      }
      else
      {
        msg_info() << "Model checked successfully";
      }
      msg_info() << "Robot nq = " << model->nq << " / nv = " << model->nv;
      msg_info() << "Robot njoints = " << model->njoints << " (incl. \"universe\") / nbodies = " << model->nbodies << " / nframes = " << model->nframes;
      msg_info() << "Robot model 6d gravity g = " << model->gravity;
      msg_info() << "Robot inertias vector size = " << model->inertias.size();

      for(pinocchio::JointIndex jointIdx = 0u; jointIdx < model->njoints; ++jointIdx)
      {
        msg_info() << "Joint[" << jointIdx << "] (index = " << model->joints[jointIdx].id() << "): " << model->names[jointIdx] << " / " << model->joints[jointIdx];
      }


      // TODO use collisionModel to create collision nodes in SOFA
      collisionModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::COLLISION, *collisionModel, modelDir);
      // msg_info() << "Built robot collision model from URDF file: " << urdf_filename;

      visualModel = std::make_shared<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(*model, urdfFilename, pinocchio::VISUAL, *visualModel, modelDir);
      msg_info() << "Built robot visual model from URDF file: " << urdfFilename;

      std::map< pinocchio::JointIndex, std::vector<std::string>> empty_jointIdToMeshMap;

      convertPinModel2SOFA(model, visualModel, empty_jointIdToMeshMap, context, d_qInit, d_qRest, d_useFreeFlyerRootJoint.getValue(), d_fixUniverse.getValue());
      return true;

    }
    catch (std::exception &e)
    {
      msg_error() << "Caught exception: " << e.what();
      d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
      return false;
    }

  }



  simulation::Node::SPtr URDFModelLoader::convertPinModel2SOFA(std::shared_ptr<pinocchio::Model> model, std::shared_ptr <pinocchio::GeometryModel> visualModel, std::map< pinocchio::FrameIndex, std::vector<std::string>>& frame2MeshesMap,
      simulation::Node* context,  Data < sofa::type::vector<sofa::type::Vec1d> > &d_qInit, Data < sofa::type::vector<sofa::type::Vec1d> > &d_qRest, bool useFreeFlyerRootJoint, bool fixUniverse)
  {
      // add a frame for each body centered on its CenterOfMass and that will be used as body DoF by SOFA
      std::vector<pinocchio::FrameIndex> body_center_of_mass_frames;
      for (pinocchio::JointIndex jointIdx = sofa::rigidbodydynamics::kSkipUniverse; jointIdx < model->njoints; ++jointIdx)
      {
          const pinocchio::SE3 body_CenterofMass_i = pinocchio::SE3(Eigen::Matrix3d::Identity(), model->inertias[jointIdx].lever());
          const auto bodyFrame_CenterOfMass = pinocchio::Frame{ "Body_" + std::to_string(jointIdx) + "_CenterOfMass", jointIdx, body_CenterofMass_i, pinocchio::FrameType::OP_FRAME };
          body_center_of_mass_frames.push_back(model->addFrame(bodyFrame_CenterOfMass));
          msg_info("convertPinModel2SOFA") << "==== bodyCenterOfMassFrames[" << jointIdx << "]: " << body_center_of_mass_frames.back();
      }
      std::vector<pinocchio::FrameIndex> extraFrames;
      // TODO should be configurable to ligthen computations
      msg_info("convertPinModel2SOFA") << "Adding all frames (num = " << model->nframes << ") to kinematic mapping...";
      for (auto frameIdx = 0u; frameIdx < model->nframes; ++frameIdx)
      {
          msg_info("convertPinModel2SOFA") << "Frame[" << frameIdx << "]: " << model->frames[frameIdx].name << " / parent Joint = " << model->frames[frameIdx].parentJoint << " / parent Frame = " << model->frames[frameIdx].parentFrame;
          extraFrames.push_back(frameIdx);
      }

      //const bool useFreeFlyerRootJoint = d_useFreeFlyerRootJoint.getValue();
      const int num_joints = model->njoints - sofa::rigidbodydynamics::kSkipUniverse; // if we do not want universe joint

      // create articulated-chain scene tree
      const simulation::Node::SPtr artichainNode = context->createChild("Articulated-chain");

      const auto jointsDofs = New<MechanicalObjectVec1>();
      jointsDofs->setName("dofs");
      auto nqWithoutRootJoint = useFreeFlyerRootJoint ? model->nq - 7 : model->nq;
      msg_info("convertPinModel2SOFA") << "nqWithoutRootJoint = " << nqWithoutRootJoint;

      if (!d_qInit.isSet())
      {
          sofa::type::Vec1d defaultDofValue;
          defaultDofValue.set(0.);
          sofa::type::vector<sofa::type::Vec1d> q0Values(nqWithoutRootJoint, defaultDofValue);
          d_qInit.setValue(q0Values);// = q0Values;
      }
      if (!d_qRest.isSet())
      {
          sofa::type::Vec1d defaultDofValue;
          defaultDofValue.set(0.);
          sofa::type::vector<sofa::type::Vec1d> q0Values(nqWithoutRootJoint, defaultDofValue);
          d_qRest.setValue(q0Values); // = q0Values;
      }
      jointsDofs->resize(nqWithoutRootJoint);
      // set initial position specified from \"qInit\" data field
      jointsDofs->x.setParent(&d_qInit);
      //jointsDofs->x.setValue(qInit);
      // set rest position specified from \"qRest\" data field
      jointsDofs->x0.setParent(&d_qRest);
      //jointsDofs->x0.setValue(qRest);

      artichainNode->addObject(jointsDofs);


       //if(d_addJointsActuators.getValue() == true)
       {
         for(JointIndex jointIdx = 1; jointIdx < model->njoints; ++jointIdx)
         {
           msg_info("convertPinModel2SOFA") << "model->joints["<<jointIdx<<"].idx_q(): " << model->joints[jointIdx].idx_q() << " and model->joints[jointIdx].nq(): " << model->joints[jointIdx].nq();

           const auto& joint = model->joints[jointIdx];
           const int jointLimitId = joint.idx_q();
           const int jointNbDofs = joint.nq();
           msg_info("convertPinModel2SOFA") << "** joint[" << jointIdx << "]: " << joint.classname() << " / shortname: " << joint.shortname();
		   msg_info("convertPinModel2SOFA") << "** joint[" << jointIdx << "]: low = " << model->lowerPositionLimit[jointLimitId];
           //if(joint.shortname().rfind("JointModelSphericalZYX", 0) == 0)
           if(model->lowerPositionLimit[jointLimitId] != std::numeric_limits<double>::min()  || model->upperPositionLimit[jointLimitId] != std::numeric_limits<double>::max())
           {
            int startConstraintId = joint.idx_q();
            for (size_t j = 0; j < jointNbDofs; j++)
            {
                const auto jointConstraint = New<softrobots::constraint::JointConstraint<sofa::defaulttype::Vec1Types>>();
                jointConstraint->setName("constraint_" + model->names[jointIdx]+"_"+std::to_string(j));
                jointConstraint->d_index = startConstraintId + j;// 3 * (jointIdx - 1) + j;//jointLimitId + j;
                jointConstraint->d_minDisplacement = model->lowerPositionLimit[jointLimitId + j];
                jointConstraint->d_maxDisplacement = model->upperPositionLimit[jointLimitId + j];
                artichainNode->addObject(jointConstraint);
                msg_info("convertPinModel2SOFA") << "****** added joint constraint for joint[" << jointIdx << "] (name = " << model->names[jointIdx] << ") with index = " << jointConstraint->d_index;
                msg_info("convertPinModel2SOFA") << "****** d_minDisplacement " << jointConstraint->d_minDisplacement << "  and d_maxDisplacement "<< jointConstraint->d_maxDisplacement;
            }
           }
         }
       }

      const auto modelNode = artichainNode->createChild("Model");

      // create mapping between articulated-chain joints dofs and its bodies placements
      const auto kinematicChainMapping = New<sofa::component::mapping::nonlinear::KinematicChainMapping<Vec1Types, Rigid3Types, Rigid3Types>>();
      kinematicChainMapping->setName("kinematicChainMapping");
      kinematicChainMapping->set_body_centerOfMass_frames(body_center_of_mass_frames);
      kinematicChainMapping->m_extraFrames = extraFrames;
      // kinematicChainMapping->f_printLog.setValue(true);
      kinematicChainMapping->setModel(model);

      // set mapping input1
      kinematicChainMapping->addInputModel1(jointsDofs.get());

      // set mapping output
      // one dof container for all joints and extra frames
      const auto mappedDof = New<MechanicalObjectRigid3>();
      mappedDof->setName("mappedDof");
      mappedDof->resize(num_joints + extraFrames.size());
      modelNode->addObject(mappedDof);

      kinematicChainMapping->addOutputModel(mappedDof.get());

      modelNode->addObject(kinematicChainMapping);

      // set joints
      const auto jointsNode = modelNode->createChild("Joints");
      const auto jointsDof = New<MechanicalObjectRigid3>();
      jointsDof->setName("jointsDof");
      jointsDof->resize(num_joints);
      jointsNode->addObject(jointsDof);
      const auto jointsMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
      jointsMapping->setName("jointsMapping");
      jointsMapping->setModels(mappedDof.get(), jointsDof.get());
      std::vector<unsigned int> jointsDofIndexes;
      for (JointIndex i = 0; i < num_joints; ++i)
      {
          jointsDofIndexes.push_back(i);
      }
      jointsMapping->d_rigidIndexPerPoint = jointsDofIndexes;
      jointsMapping->d_globalToLocalCoords = false;
      jointsNode->addObject(jointsMapping);

      // set frames
      const auto framesNode = modelNode->createChild("Frames");
      const auto framesDof = New<MechanicalObjectRigid3>();
      framesDof->setName("framesDof");
      framesDof->resize(extraFrames.size());
      framesNode->addObject(framesDof);
      const auto framesMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
      framesMapping->setName("framesMapping");
      framesMapping->setModels(mappedDof.get(), framesDof.get());
      std::vector<unsigned int> framesDofIndexes;
      for (pinocchio::FrameIndex frameIdx = pinocchio::FrameIndex(0); frameIdx < extraFrames.size(); ++frameIdx)
      {
          framesDofIndexes.push_back(num_joints + frameIdx);
          // create a node + submapping for each frame
          const auto& frame = model->frames[extraFrames[frameIdx]];

		  std::string frameName = frame.name;
          if(frameName[0] == '/')
			  frameName = "'" + frameName + "'";
          const auto frameNode = framesNode->createChild(frameName);
          const auto frameDof = New<MechanicalObjectRigid3>();
          frameDof->setName("dof");
          frameNode->addObject(frameDof);

          if (fixUniverse && frameIdx == pinocchio::FrameIndex(0))
          {
              //const auto fixedConstraint = New < sofa::component::constraint::projective::FixedProjectiveConstraint<Rigid3Types>>();
              const auto fixedConstraint = New < sofa::component::constraint::lagrangian::model::FixedLagrangianConstraint<Rigid3Types>>();
			  fixedConstraint-> setName("fixedConstraint");
              fixedConstraint->d_indices.setValue(sofa::topology::SetIndex(1, 0));
			  frameNode->addObject(fixedConstraint);
              //const auto constraintCorrection = New < sofa::component::constraint::lagrangian::correction::LinearSolverConstraintCorrection<Rigid3Types>>();
			  //constraintCorrection->setName("constraintCorrection");
              //frameNode->addObject(constraintCorrection);
          }

          const auto frameMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
          frameMapping->setName("frameMapping");
          frameMapping->setModels(framesDof.get(), frameDof.get());
          frameMapping->d_index = sofa::Index(frameIdx);
		  frameMapping->setMatricesMapped(true);
          frameNode->addObject(frameMapping);


          if (frame2MeshesMap.find(frameIdx) != frame2MeshesMap.end())
          {
			  const std::vector<std::string>& meshFiles = frame2MeshesMap.at(frameIdx);
              for (size_t mfId = 0; mfId < meshFiles.size(); mfId++)
              {
                  const auto visualNode = frameNode->createChild("Visual_"+ std::filesystem::path(meshFiles[mfId]).stem().string());
                  msg_info("convertPinModel2SOFA") << "mesh file extension: " << std::filesystem::path(meshFiles[mfId]).extension().string();

                  sofa::core::loader::MeshLoader::SPtr loader;
                  if (std::filesystem::path(meshFiles[mfId]).extension().string() == ".obj") {
                      loader = New<component::io::mesh::MeshOBJLoader>();
                  }
                  else if (std::filesystem::path(meshFiles[mfId]).extension().string() == ".vtp"
                           || std::filesystem::path(meshFiles[mfId]).extension().string() == ".vtk") {
                      loader = New<component::io::mesh::MeshVTKLoader>();
                  }
                  else
                  {
                      msg_error("convertPinModel2SOFA") << "mesh file " << "has extension not readable by the loader, abort";
                      return nullptr;
                  }
                  loader->setName("MeshLoader");
                  loader->d_filename.setValue(meshFiles[mfId]);
                  visualNode->addObject(loader);
                  loader->reinit();

                  const auto visualMeshTopology = sofa::core::objectmodel::New<sofa::component::topology::container::constant::MeshTopology>();
                  visualMeshTopology->setName("visualTopology");

                  for (sofa::type::Vec3d pt : loader->d_positions.getValue())
                  {
                      visualMeshTopology->addPoint(pt[0], pt[1], pt[2]);
                  }
                  for (core::topology::BaseMeshTopology::Edge ed : loader->d_edges.getValue())
                  {
                      visualMeshTopology->addEdge(ed[0], ed[1]);
                  }
                  for (core::topology::BaseMeshTopology::Triangle tri : loader->d_triangles.getValue())
                  {
                      visualMeshTopology->addTriangle(tri[0], tri[1], tri[2]);
                  }
                  for (core::topology::BaseMeshTopology::Quad quad : loader->d_quads.getValue())
                  {
                      visualMeshTopology->addQuad(quad[0], quad[1], quad[2], quad[3]);
                  }

                  visualNode->addObject(visualMeshTopology);

                  // add visual openGL model
                  auto visualBodyModel = New<sofa::gl::component::rendering3d::OglModel>();
                  visualBodyModel->setName("visualModel");
                  visualBodyModel->l_topology = visualMeshTopology;
                  visualBodyModel->setColor(1.0, 1.0, 1.0, 1.0);
                  visualNode->addObject(visualBodyModel);

                  const auto visualMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Vec3Types>>();
                  visualMapping->setName("visualMapping");
                  visualMapping->setModels(frameDof.get(), visualBodyModel.get());
                  visualMapping->f_mapConstraints.setValue(false);
                  visualMapping->f_mapForces.setValue(false);
                  visualMapping->f_mapMasses.setValue(false);
                  visualMapping->d_globalToLocalCoords = false;
                  visualMapping->f_applyRestPosition = true;

                  visualNode->addObject(visualMapping);
              }
          }


      }
      framesMapping->d_rigidIndexPerPoint = framesDofIndexes;
      framesMapping->d_globalToLocalCoords = false;
      framesNode->addObject(framesMapping);

      // set mapping input2: free flyer root joint if any specified
      if (useFreeFlyerRootJoint)
      {
          const auto rootJointNode = context->createChild("RootJoint");

          const auto rootJointDof = New<MechanicalObjectRigid3>();
          rootJointDof->setName("Free-Flyer");
          rootJointDof->resize(1);
          rootJointNode->addObject(rootJointDof);

          // Joints node have two parents: rootJointNode and artichainNode
          rootJointNode->addChild(jointsNode);

          // set mapping input2
          kinematicChainMapping->addInputModel2(rootJointDof.get());
      }

      msg_info("convertPinModel2SOFA") << "-- pinocchio model num bodies: " << model->nbodies;
      msg_info("convertPinModel2SOFA") << "-- pinocchio model num joints: " << model->njoints;
      msg_info("convertPinModel2SOFA") << "-- SOFA converted model num_joints : " << num_joints;

      for (JointIndex jointIdx = 0; jointIdx < num_joints; ++jointIdx)
      {
          const pinocchio::JointIndex pinoJointIdx = jointIdx + sofa::rigidbodydynamics::kSkipUniverse;

          msg_info("convertPinModel2SOFA") << "-- pinocchio joint idx: " << pinoJointIdx;
          msg_info("convertPinModel2SOFA") << "-- joint name " << model->names[pinoJointIdx];
          msg_info("convertPinModel2SOFA") << "-- joint shortname " << model->joints[pinoJointIdx].shortname();

          std::string jointName = model->names[pinoJointIdx];
          if (jointName[0] == '/')
              jointName = "'" + jointName + "'";

          const auto jointNode = jointsNode->createChild(jointName);
          const auto& jointInertia = model->inertias[pinoJointIdx];

          const auto bodyRigid = New<MechanicalObjectRigid3>();
          bodyRigid->setName("jointRigid");
          const Eigen::Vector3d inv_body_centerOfMass_translation = -jointInertia.lever();
          bodyRigid->setTranslation(inv_body_centerOfMass_translation.x(), inv_body_centerOfMass_translation.y(), inv_body_centerOfMass_translation.z());
          jointNode->addObject(bodyRigid);

          const auto bodyMass = New<sofa::component::mass::UniformMass<Rigid3Types>>();
          bodyMass->setName("mass");
          sofa::defaulttype::Rigid3dMass rigidMass;
          rigidMass.mass = jointInertia.mass();
          const Eigen::Matrix3d massInertia = jointInertia.inertia().matrix();
          const Eigen::Matrix3d inertiaDivByMass = massInertia / rigidMass.mass;
          msg_info("convertPinModel2SOFA") << "-- rigidMass.mass = " << rigidMass.mass;
          msg_info("convertPinModel2SOFA") << "-- massInertia = " << massInertia;
          msg_info("convertPinModel2SOFA") << "-- inertiaDivByMass = " << inertiaDivByMass;

          if (rigidMass.mass != 0)
              rigidMass.inertiaMatrix = sofa::rigidbodydynamics::mat3ToSofaType(inertiaDivByMass);

          rigidMass.volume = 1.; // XXX: should not be used here as we only deal with rigid bodies, so we should be able to set any value

          if (equalsZero(rigidMass.mass) || equalsZero(determinant(rigidMass.inertiaMatrix)) || equalsZero(determinant(rigidMass.inertiaMatrix * rigidMass.mass)))
          {
              rigidMass.mass = 0.1;
              rigidMass.inertiaMatrix = type::Mat<3, 3, SReal>::Identity();
              msg_info("convertPinModel2SOFA") << "-- new rigidMass.inertiaMatrix " << rigidMass.inertiaMatrix;
          }
          rigidMass.recalc();
          bodyMass->setMass(rigidMass);
          bodyMass->d_showAxisSize.setValue(0.08);
          jointNode->addObject(bodyMass);

          const auto bodyMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Rigid3Types>>();
          bodyMapping->setName("jointMapping");
          bodyMapping->setModels(mappedDof.get(), bodyRigid.get());
          bodyMapping->d_index = jointIdx;
          bodyMapping->d_globalToLocalCoords = false;
          jointNode->addObject(bodyMapping);


          if (visualModel)
          {
              // add visual body node
              const auto visualNode = jointNode->createChild("Visual");

              // get joint associated visual geometries
              const auto visualData = std::make_shared<pinocchio::GeometryData>(*visualModel);
              const auto visualGeomIndexesIt = visualData->innerObjects.find(pinoJointIdx);
              if (visualGeomIndexesIt != visualData->innerObjects.end())
              {
                  for (const auto& geomIdx : visualGeomIndexesIt->second)
                  {
                      const auto& geom = visualModel->geometryObjects[geomIdx];

                      const auto visualBodyNode = visualNode->createChild(geom.name);
                      // msg_info() << "joint[" << jointIdx << "]:geom name: " << geom.name << " / parent joint: " << geom.parentJoint << " / object type: " << static_cast<int>(geom.geometry->getObjectType()) << " / node type: " << static_cast<int>(geom.geometry->getNodeType());
                      // msg_info() << "overrideMaterial: " << geom.overrideMaterial << " / mesh color: " << geom.meshColor;
                      // msg_info() << "meshPath: " << geom.meshPath;
                      // msg_info() << "meshTexturePath: " << geom.meshTexturePath;

                      auto visualBodyMesh = sofa::rigidbodydynamics::fclGeometryToSofaTopology(geom.geometry, geom.placement, geom.meshScale);
                      if (!visualBodyMesh)
                      {
                          msg_error("convertPinModel2SOFA visual") << "Failed to convert pinocchio FCL visual geometry to Sofa MeshTopology";
                          msg_error("convertPinModel2SOFA visual" ) << "FCL geometry object type: " << static_cast<int>(geom.geometry->getObjectType()) << ", FCL geometry node type: " << static_cast<int>(geom.geometry->getNodeType());
                          return nullptr;
                      }
                      visualBodyNode->addObject(visualBodyMesh);

                      // add visual openGL model
                      auto visualBodyModel = New<sofa::gl::component::rendering3d::OglModel>();
                      // visualBodyModel->f_printLog = true;
                      visualBodyModel->setName("visualModel");
                      visualBodyModel->l_topology = visualBodyMesh;
                      visualBodyModel->setColor(geom.meshColor[0], geom.meshColor[1], geom.meshColor[2], geom.meshColor[3]);
                      visualBodyNode->addObject(visualBodyModel);

                      const auto visualMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Vec3Types>>();
                      visualMapping->setName("visualMapping");
                      visualMapping->setModels(bodyRigid.get(), visualBodyModel.get());
                      visualMapping->f_mapConstraints.setValue(false);
                      visualMapping->f_mapForces.setValue(false);
                      visualMapping->f_mapMasses.setValue(false);
                      visualBodyNode->addObject(visualMapping);
                  }
              }
          }

          //// add collision body node
          //if (d_addCollision.getValue() == true)
          //{
          //    const auto collisionNode = jointNode->createChild("Collision");
          //    const auto collisionData = std::make_shared<pinocchio::GeometryData>(*collisionModel);
          //    const auto collisionGeomIndexesIt = collisionData->innerObjects.find(pinoJointIdx);
          //    if (collisionGeomIndexesIt != collisionData->innerObjects.end())
          //    {
          //        for (const auto& geomIdx : collisionGeomIndexesIt->second)
          //        {
          //            const auto& geom = collisionModel->geometryObjects[geomIdx];

          //            const auto collisionBodyNode = collisionNode->createChild(geom.name);

          //            auto collisionBodyMesh = sofa::rigidbodydynamics::fclGeometryToSofaTopology(geom.geometry, geom.placement, geom.meshScale);
          //            if (!collisionBodyMesh)
          //            {
          //                msg_error() << "Failed to convert pinocchio FCL collision geometry to Sofa MeshTopology";
          //                msg_error() << "FCL geometry object type: " << static_cast<int>(geom.geometry->getObjectType()) << ", FCL geometry node type: " << static_cast<int>(geom.geometry->getNodeType());
          //                d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
          //                return false;
          //            }
          //            collisionBodyNode->addObject(collisionBodyMesh);

          //            auto collisionBodyObj = New<sofa::component::statecontainer::MechanicalObject<Vec3Types>>();
          //            collisionBodyObj->setName("collisionMecObject");
          //            // collisionBodyObj->l_topology = collisionBodyMesh;
          //            collisionBodyNode->addObject(collisionBodyObj);

          //            const auto collisionMapping = New<sofa::component::mapping::nonlinear::RigidMapping<Rigid3Types, Vec3Types>>();
          //            collisionMapping->setName("collisionMapping");
          //            collisionMapping->setModels(bodyRigid.get(), collisionBodyObj.get());
          //            collisionBodyNode->addObject(collisionMapping);

          //            // add collision model
          //            auto collisionTriModel = New<sofa::component::collision::geometry::TriangleCollisionModel<Vec3Types>>();
          //            // collisionTriModel->f_printLog.setValue(true);
          //            collisionTriModel->setName("collisionTriModel");
          //            collisionTriModel->l_topology = collisionBodyMesh;
          //            collisionTriModel->addGroup(1);
          //            collisionBodyNode->addObject(collisionTriModel);

          //            // auto collisionLineModel = New<sofa::component::collision::geometry::LineCollisionModel<Vec3Types>>();
          //            // collisionLineModel->setName("collisionLineModel");
          //            // collisionLineModel->l_topology = collisionBodyMesh;
          //            // collisionLineModel->addGroup(1);
          //            // collisionBodyNode->addObject(collisionLineModel);

          //            // auto collisionPointModel = New<sofa::component::collision::geometry::PointCollisionModel<Vec3Types>>();
          //            // collisionPointModel->setName("collisionPointModel");
          //            // // collisionPointModel->l_topology = collisionBodyMesh;
          //            // collisionPointModel->addGroup(1);
          //            // collisionBodyNode->addObject(collisionPointModel);
          //        }
          //    }
          //}
      }
      msg_info("convertPinModel2SOFA") << "Model has " << model->referenceConfigurations.size() << " reference configurations registered";

      return artichainNode;
  }

} /// namespace sofa::rigidbodydynamics

int URDFModelLoaderClass = sofa::core::RegisterObject("Loads robot from URDF file.").add<sofa::rigidbodydynamics::URDFModelLoader>();