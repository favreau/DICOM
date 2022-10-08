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

#include "DICOMPlugin.h"
#include "log.h"

#include <brayns/common/ActionInterface.h>
#include <brayns/common/Progress.h>
#include <brayns/engineapi/Camera.h>
#include <brayns/engineapi/Material.h>
#include <brayns/engineapi/Scene.h>
#include <brayns/parameters/ParametersManager.h>
#include <brayns/pluginapi/Plugin.h>

#define REGISTER_LOADER(LOADER, FUNC) \
    registry.registerLoader({std::bind(&LOADER::getSupportedDataTypes), FUNC});

DICOMPlugin::DICOMPlugin(brayns::PropertyMap &&dicomParams)
    : ExtensionPlugin()
    , _dicomParams(dicomParams)
{
}

void DICOMPlugin::init()
{
    auto &scene = _api->getScene();
    auto &registry = scene.getLoaderRegistry();
    registry.registerLoader(std::make_unique<DICOMLoader>(
        scene, std::move(_api->getParametersManager().getGeometryParameters()),
        std::move(_dicomParams)));
}

extern "C" brayns::ExtensionPlugin *brayns_plugin_create(int /*argc*/,
                                                         char ** /*argv*/)
{
    return new DICOMPlugin(DICOMLoader::getCLIProperties());
}
