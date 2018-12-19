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

#include <io/DICOMLoader.h>

#include <brayns/common/types.h>
#include <brayns/pluginapi/ExtensionPlugin.h>

/**
 * @brief The AtomicVolumesPlugin class manages the loading of
 * RAW volumes as point clouds
 */
class DICOMPlugin : public brayns::ExtensionPlugin
{
public:
    DICOMPlugin(brayns::PropertyMap&& dicomParams);

    void init() final;

private:
    brayns::PropertyMap _dicomParams;
    bool _dirty{false};
};

#endif
