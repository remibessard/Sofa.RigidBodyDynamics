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
#pragma once

#include <sofa/RigidBodyDynamics/config.h>
#include <sofa/RigidBodyDynamics/Types.h>

#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/Node.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/default-check.hpp>


namespace sofa::rigidbodydynamics
{

  using namespace core::objectmodel;

  class SOFA_RIGIDBODYDYNAMICS_API URDFModelLoader : public sofa::core::loader::SceneLoader
  {
  public:
    SOFA_CLASS(URDFModelLoader, sofa::core::loader::SceneLoader);

    void setModelDirectory(const std::string &d);
    const std::string &getModelDirectory() const /*const*/;

    void setUseFreeFlyerRootJoint(bool useFreeFlyerRootJoint);
    bool getUseFreeFlyerRootJoint() const;

    int kSkipUniverse = 1;


    void init() override;

    bool load() override;

    static simulation::Node::SPtr convertPinModel2SOFA(std::shared_ptr<pinocchio::Model> model, std::shared_ptr <pinocchio::GeometryModel> visualModel, std::map< pinocchio::FrameIndex, std::vector<std::string>>& frame2MeshesMap,
        simulation::Node* context, Data < sofa::type::vector<sofa::type::Vec1d> > &qInit, Data < sofa::type::vector<sofa::type::Vec1d> > &qRest, bool useFreeFlyerRootJoint = false, bool fixUniverse = false);

    Data<std::string> d_modelDirectory;
    Data<bool> d_useFreeFlyerRootJoint;
    Data<bool> d_addCollision;
    Data<bool> d_addJointsActuators;
	Data<bool> d_fixUniverse; // if true, adds a FixedConstraint to universe Frame
    Data<sofa::type::vector<sofa::type::Vec1d>> d_qRest;
    Data<sofa::type::vector<sofa::type::Vec1d>> d_qInit;

  private:
    URDFModelLoader();
  };

} /// namespace sofa::rigidbodydynamics