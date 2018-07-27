/* Copyright (c) 2018, Cyrille Favreau
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Cyrille Favreau <cyrille.favreau@gmail.com>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef DICOMPLUGIN_H
#define DICOMPLUGIN_H

#include "DICOMLoader.h"

#include <array>
#include <brayns/common/types.h>
#include <brayns/pluginapi/ExtensionPlugin.h>
#include <vector>

/**
 * @brief The AtomicVolumesPlugin class manages the loading of
 * RAW volumes as point clouds
 */
class DICOMPlugin : public brayns::ExtensionPlugin
{
public:
    DICOMPlugin(brayns::Scene& scene,
                brayns::ParametersManager& parametersManager,
                brayns::ActionInterface* actionInterface,
                brayns::Camera& camera, int argc, char** argv);

    /**
     * @brief preRender Updates the scene according to latest data load
     */
    void preRender() final;

private:
    brayns::Scene& _scene;
    brayns::ParametersManager& _parametersManager;
    brayns::Camera& _camera;
};

#endif
