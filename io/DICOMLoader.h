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
    brayns::DataType dataType;
    brayns::Vector2ui dimensions{0, 0};
    brayns::Vector3f position{0, 0, 0};
    brayns::Vector2f pixelSpacing{1, 1};
    brayns::Vector2f dataRange;
    std::vector<char> buffer;
    uint16_t nbFrames;
};
using DICOMImageDescriptors = std::vector<DICOMImageDescriptor>;

class DICOMLoader : public brayns::Loader
{
public:
    DICOMLoader(brayns::Scene& scene,
                const brayns::GeometryParameters& geometryParameters,
                brayns::PropertyMap&& loaderParams);

    std::string getName() const final;

    std::vector<std::string> getSupportedExtensions() const final;

    bool isSupported(const std::string& filename,
                     const std::string& extension) const final;

    static brayns::PropertyMap getCLIProperties();

    brayns::ModelDescriptorPtr importFromFile(
        const std::string& path, const brayns::LoaderProgress& callback,
        const brayns::PropertyMap& properties, const size_t index = 0,
        const size_t defaultMaterial = brayns::NO_MATERIAL) const final;

    brayns::ModelDescriptorPtr importFromBlob(
        brayns::Blob&& blob, const brayns::LoaderProgress& callback,
        const brayns::PropertyMap& properties, const size_t index = 0,
        const size_t defaultMaterial = brayns::NO_MATERIAL) const final;

    brayns::ModelDescriptorPtr importFromFolder(const std::string& path);

private:
    void _setDefaultTransferFunction(brayns::Model& model,
                                     const brayns::Vector2f& dataRange) const;

    std::string _dataTypeToString(const brayns::DataType& dataType) const;

    DICOMImageDescriptors _parseDICOMImagesData(
        const std::string& fileName, brayns::ModelMetadata& metadata) const;

    void _readDICOMFile(const std::string& path,
                        DICOMImageDescriptor& imageDescriptor) const;

    brayns::ModelDescriptorPtr _readDirectory(
        const std::string& path, const brayns::LoaderProgress& callback) const;

    brayns::ModelDescriptorPtr _readFile(const std::string& path) const;

    const brayns::GeometryParameters& _geometryParameters;
    brayns::PropertyMap _loaderParams;
};

#endif // DICOMLOADER_H
