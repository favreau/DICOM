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
#include <brayns/common/camera/Camera.h>
#include <brayns/common/material/Material.h>
#include <brayns/common/scene/Scene.h>
#include <brayns/parameters/ParametersManager.h>
#include <brayns/pluginapi/PluginAPI.h>

#define REGISTER_LOADER(LOADER, FUNC) \
    registry.registerLoader({std::bind(&LOADER::getSupportedDataTypes), FUNC});

DICOMPlugin::DICOMPlugin(brayns::Scene &scene,
                         brayns::ParametersManager &parametersManager,
                         brayns::ActionInterface *actionInterface,
                         brayns::Camera &camera, int /*argc*/, char ** /*argv*/)
    : ExtensionPlugin()
    , _scene(scene)
    , _parametersManager(parametersManager)
    , _camera(camera)
{
    auto &registry = _scene.getLoaderRegistry();
    REGISTER_LOADER(DICOMLoader,
                    ([
                           &scene = _scene,
                           &params = _parametersManager.getGeometryParameters()
                    ] {
                        return std::make_unique<DICOMLoader>(scene, params);
                    }));

    if (actionInterface)
    {
        actionInterface->registerNotification<DICOMFolder>(
            "dicom",
            [&](const DICOMFolder &s) { _updateDICOMFolderFromJson(s); });
    }
}

void DICOMPlugin::_updateDICOMFolderFromJson(const DICOMFolder &folder)
{
    _folder = folder;
    _dirty = true;
}

void DICOMPlugin::preRender()
{
    if (_dirty)
    {
        PLUGIN_INFO << "Importing DICOM data from " << _folder.path
                    << std::endl;
        DICOMLoader loader(_scene, _parametersManager.getGeometryParameters());
        _scene.addModel(loader.importFromFolder(_folder.path));
    }
    _dirty = false;
}

extern "C" brayns::ExtensionPlugin *brayns_plugin_create(brayns::PluginAPI *api,
                                                         int argc, char **argv)
{
    return new DICOMPlugin(api->getScene(), api->getParametersManager(),
                           api->getActionInterface(), api->getCamera(), argc,
                           argv);
}
