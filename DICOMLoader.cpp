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

#include "DICOMLoader.h"
#include "log.h"

#include <brayns/common/scene/Model.h>
#include <brayns/common/scene/Scene.h>
#include <brayns/common/volume/SharedDataVolume.h>

#include <dcmtk/dcmdata/dcddirif.h>
#include <dcmtk/dcmdata/dctk.h>
#include <dcmtk/dcmimgle/dcmimage.h>

#include <boost/filesystem.hpp>

template <size_t M, typename T>
std::string to_string(const vmml::vector<M, T>& vec)
{
    std::stringstream stream;
    stream << vec;
    return stream.str();
}

DICOMLoader::DICOMLoader(brayns::Scene& scene,
                         const brayns::GeometryParameters& geometryParameters)
    : Loader(scene)
    , _geometryParameters(geometryParameters)
{
}

std::set<std::string> DICOMLoader::getSupportedDataTypes()
{
    return {"", "dicom", "dcm"};
}

DICOMImageDescriptors DICOMLoader::parseDICOMImagesData(
    const std::string& fileName, std::string& patientName)
{
    DICOMImageDescriptors dicomImages;
    DcmDicomDir dicomdir(fileName.c_str());
    if (dicomdir.verify().good())
    {
        auto root = dicomdir.getRootRecord();
        //        root.print(std::cout);
        auto sub = root.getSub(0);

        // Patient name
        OFString pn;
        if (sub)
        {
            sub->findAndGetOFString(DCM_PatientName, pn);
            patientName = pn.c_str();
        }

        // Images
        while (sub)
        {
            for (size_t i = 0; i < sub->cardSub(); ++i)
            {
                auto s = sub->getSub(i);
                if (s->getRecordType() == ERT_Image)
                {
                    OFString refId;
                    s->findAndGetOFStringArray(DCM_ReferencedFileID, refId);

                    // Replace backslashes with slashes
                    std::string str = std::string(refId.data());
                    while (str.find("\\") != std::string::npos)
                        str.replace(str.find("\\"), 1, "/");

                    // Get folder from filename
                    boost::filesystem::path path = fileName;
                    boost::filesystem::path folder = path.parent_path();
                    const std::string imageFileName =
                        std::string(folder.string()) + "/" + str;

                    // Image position patient
                    double ipp[3];
                    for (size_t j = 0; j < 3; ++j)
                        s->findAndGetFloat64(DCM_ImagePositionPatient, ipp[j],
                                             j);

                    // Image size
                    long es[2];
                    s->findAndGetLongInt(DCM_Columns, es[0], 0);
                    s->findAndGetLongInt(DCM_Rows, es[1], 0);

                    // Image size
                    double ps[2];
                    for (size_t j = 0; j < 2; ++j)
                        s->findAndGetFloat64(DCM_PixelSpacing, ps[j], j);

                    dicomImages.push_back(
                        {imageFileName,
                         {(unsigned int)es[0], (unsigned int)es[1]},
                         {(float)ipp[0], (float)ipp[1], (float)ipp[2]},
                         {(float)ps[0], (float)ps[1]},
                         {}});
                }
            }

            // Next element
            sub = sub->getSub(0);
        }
    }
    else
        PLUGIN_ERROR << fileName << std::endl;
    return dicomImages;
}

DICOMImageDescriptor DICOMLoader::readDICOMFile(const std::string& fileName)
{
    DICOMImageDescriptor imageDescriptor;
    DicomImage* image = new DicomImage(fileName.c_str());
    if (image)
    {
        if (image->getStatus() != EIS_Normal)
            throw std::runtime_error("Error: cannot load DICOM image from " +
                                     fileName);
        if (image->getInterData())
        {
            if (image->getInterData()->getRepresentation() != EPR_Sint16)
                throw std::runtime_error(
                    "Only 16bit volumes are currently supported");
        }
        else
            throw std::runtime_error("Failed to identify image representation");

        imageDescriptor.dimensions = {(unsigned int)image->getWidth(),
                                      (unsigned int)image->getHeight()};
        imageDescriptor.pixelSpacing = {1, 1};
        const auto size =
            imageDescriptor.dimensions.x() * imageDescriptor.dimensions.y();
        imageDescriptor.buffer.resize(size);
        image->getOutputData(imageDescriptor.buffer.data(),
                             size * sizeof(uint16_t));
        delete image;
    }
    return imageDescriptor;
}

brayns::ModelDescriptorPtr DICOMLoader::readFile(const std::string& fileName)
{
    DICOMImageDescriptor imageDescriptor = readDICOMFile(fileName);

    // Data range
    const brayns::Vector2f dataRange = {std::numeric_limits<uint16_t>::min(),
                                        std::numeric_limits<uint16_t>::max()};

    auto volume = _scene.createSharedDataVolume(
        {imageDescriptor.dimensions.x(), imageDescriptor.dimensions.y(), 1},
        {imageDescriptor.pixelSpacing.x(), imageDescriptor.pixelSpacing.y(), 1},
        brayns::DataType::UINT16);
    auto& volumeData = volume->getData();
    volumeData.insert(volumeData.end(), imageDescriptor.buffer.begin(),
                      imageDescriptor.buffer.end());
    volume->setDataRange(dataRange);
    volume->mapData();

    // Create Model
    auto model = _scene.createModel();
    model->addVolume(volume);

    brayns::Transformation transformation;
    transformation.setRotationCenter(model->getBounds().getCenter());
    brayns::ModelMetadata metaData = {
        {"dimensions", to_string(imageDescriptor.dimensions)},
        {"element-spacing", to_string(imageDescriptor.pixelSpacing)}};

    auto modelDescriptor =
        std::make_shared<brayns::ModelDescriptor>(std::move(model), fileName,
                                                  metaData);
    modelDescriptor->setTransformation(transformation);
    return modelDescriptor;
}

brayns::ModelDescriptorPtr DICOMLoader::readDirectory(
    const std::string& fileName)
{
    std::string patientName;
    const auto& dicomImages = parseDICOMImagesData(fileName, patientName);

    if (dicomImages.empty())
        throw std::runtime_error("DICOM folder does not contain any images");

    // Data range
    const brayns::Vector2f dataRange = {std::numeric_limits<uint16_t>::min(),
                                        std::numeric_limits<uint16_t>::max()};

    // Dimensions
    brayns::Vector3ui dimensions = {dicomImages[0].dimensions.x(),
                                    dicomImages[0].dimensions.y(),
                                    (unsigned int)dicomImages.size()};

    // Element spacing (if single image, assume that z pixel spacing is the
    // same as y
    brayns::Vector3f elementSpacing{dicomImages[0].pixelSpacing.x(),
                                    dicomImages[0].pixelSpacing.y(),
                                    dicomImages[0].pixelSpacing.y()};
    if (dicomImages.size() > 1)
        elementSpacing.z() =
            dicomImages[1].position.z() - dicomImages[0].position.z();

    // Load images into volume
    updateProgress("Loading voxels ...", 1, 2);

    auto volume = _scene.createSharedDataVolume(dimensions, elementSpacing,
                                                brayns::DataType::UINT16);
    auto& volumeData = volume->getData();
    for (const auto& dicomImage : dicomImages)
    {
        auto imageDescriptor = readDICOMFile(dicomImage.path);
        volumeData.insert(volumeData.end(), imageDescriptor.buffer.begin(),
                          imageDescriptor.buffer.end());
    }
    volume->setDataRange(dataRange);
    volume->mapData();

    // Create Model
    updateProgress("Creating model ...", 2, 2);
    auto model = _scene.createModel();
    model->addVolume(volume);

    brayns::Transformation transformation;
    transformation.setRotationCenter(model->getBounds().getCenter());
    brayns::ModelMetadata metaData = {{"patient-name", patientName},
                                      {"dimensions", to_string(dimensions)},
                                      {"element-spacing",
                                       to_string(elementSpacing)}};

    auto modelDescriptor =
        std::make_shared<brayns::ModelDescriptor>(std::move(model), "DICOMDIR",
                                                  metaData);
    modelDescriptor->setTransformation(transformation);
    return modelDescriptor;
}

brayns::ModelDescriptorPtr DICOMLoader::importFromFile(
    const std::string& path, const size_t /*index*/,
    const size_t /*defaultMaterial*/)
{
    const auto extension = boost::filesystem::extension(path);
    if (extension == ".dcm")
        return readFile(path);
    return readDirectory(path);
}

brayns::ModelDescriptorPtr DICOMLoader::importFromBlob(brayns::Blob&&,
                                                       const size_t,
                                                       const size_t)
{
    throw std::runtime_error("Loading DICOM from blob is not supported");
}
