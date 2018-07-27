/* Copyright (c) 2018, Cyrille Favreau
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Cyrille Favreau <cyrille.favreau@gmail.com>
 *
 * This file is part of Brayns
 * <https://github.com/favreau/Brayns-UC-AtomicVolume>
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

#ifndef DICOMLOADER_H
#define DICOMLOADER_H

#include <brayns/common/loader/Loader.h>
#include <brayns/common/types.h>
#include <brayns/parameters/GeometryParameters.h>

#include <set>

struct DICOMImageDescriptor
{
    std::string path;
    brayns::Vector2ui size;
    brayns::Vector3f position;
    brayns::Vector2f pixelSpacing;
};
typedef std::vector<DICOMImageDescriptor> DICOMImageDescriptors;

class DICOMLoader : public brayns::Loader
{
public:
    DICOMLoader(brayns::Scene& scene,
                const brayns::GeometryParameters& geometryParameters);

    static std::set<std::string> getSupportedDataTypes();

    brayns::ModelDescriptorPtr importFromFile(
        const std::string& fileName, const size_t index = 0,
        const size_t defaultMaterial = brayns::NO_MATERIAL) final;

    brayns::ModelDescriptorPtr importFromBlob(
        brayns::Blob&& blob, const size_t index = 0,
        const size_t defaultMaterial = brayns::NO_MATERIAL) final;

private:
    DICOMImageDescriptors parseDICOMImagesData(const std::string& fileName,
                                               std::string& patientName);
    const brayns::GeometryParameters& _geometryParameters;
};

#endif // DICOMLOADER_H
