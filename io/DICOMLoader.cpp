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
#include <brayns/common/utils/Utils.h>
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

void DICOMLoader::readDICOMFile(const std::string& fileName,
                                DICOMImageDescriptor& imageDescriptor)
{
    DcmFileFormat file;
    file.loadFile(fileName.c_str());
    DcmDataset* dataset = file.getDataset();
    //    dataset->print(std::cout);
    double position[3];
    for (size_t i = 0; i < 3; ++i)
        dataset->findAndGetFloat64(DCM_ImagePositionPatient, position[i], i);
    long imageSize[2];
    dataset->findAndGetLongInt(DCM_Columns, imageSize[0], 0);
    dataset->findAndGetLongInt(DCM_Rows, imageSize[1], 0);
    double pixelSpacing[2];
    for (size_t i = 0; i < 2; ++i)
        dataset->findAndGetFloat64(DCM_PixelSpacing, pixelSpacing[i], i);

    DicomImage* image = new DicomImage(fileName.c_str());
    if (image)
    {
        imageDescriptor.nbFrames = image->getNumberOfFrames();
        PLUGIN_DEBUG << fileName << ": " << imageDescriptor.nbFrames
                     << " frame(s)" << std::endl;

        if (image->getStatus() != EIS_Normal)
            throw std::runtime_error("Error: cannot load DICOM image from " +
                                     fileName);
        const auto imageDepth = image->getDepth();
        size_t voxelSize;
        if (image->getInterData())
        {
            switch (image->getInterData()->getRepresentation())
            {
            case EPR_Sint8:
                imageDescriptor.dataType = brayns::DataType::UINT8;
                voxelSize = sizeof(int8_t);
                break;
            case EPR_Uint8:
                imageDescriptor.dataType = brayns::DataType::UINT8;
                voxelSize = sizeof(uint8_t);
                break;
            case EPR_Sint16:
                imageDescriptor.dataType = brayns::DataType::INT16;
                voxelSize = sizeof(int16_t);
                break;
            case EPR_Uint16:
                imageDescriptor.dataType = brayns::DataType::UINT16;
                voxelSize = sizeof(uint16_t);
                break;
            default:
                throw std::runtime_error("Unsupported volume format");
            }
        }
        else
            throw std::runtime_error("Failed to identify image representation");

        imageDescriptor.position = {(float)position[0], (float)position[1],
                                    (float)position[2]};
        imageDescriptor.dimensions = {(unsigned int)imageSize[0],
                                      (unsigned int)imageSize[1]};
        imageDescriptor.pixelSpacing = {(float)pixelSpacing[0],
                                        (float)pixelSpacing[1]};

        std::vector<char> rawBuffer;
        auto rawBufferSize = image->getOutputDataSize();
        rawBuffer.resize(rawBufferSize);

        if (!image->getOutputData(rawBuffer.data(), rawBufferSize, imageDepth))
            throw std::runtime_error("Failed to load image data from " +
                                     fileName);

        // Convert frame buffer according to bit depth
        size_t step = 1;
        if (float(imageDepth) / float(8 * voxelSize) > 1.f)
            step = 2;
        for (size_t i = 0; i < rawBufferSize; ++i)
            if ((i / voxelSize) % step == 0)
                imageDescriptor.buffer.push_back(rawBuffer[i]);

        double minRange, maxRange;
        image->getMinMaxValues(minRange, maxRange);
        imageDescriptor.dataRange = {float(minRange), float(maxRange)};
        delete image;
    }
    else
        throw std::runtime_error("Failed to open " + fileName);
}

DICOMImageDescriptors DICOMLoader::parseDICOMImagesData(
    const std::string& fileName, std::string& patientName)
{
    DICOMImageDescriptors dicomImages;
    DcmDicomDir dicomdir(fileName.c_str());
    DcmDirectoryRecord* studyRecord = nullptr;
    DcmDirectoryRecord* patientRecord = nullptr;
    DcmDirectoryRecord* seriesRecord = nullptr;
    DcmDirectoryRecord* imageRecord = nullptr;
    OFString tmpString;

    if (!dicomdir.verify().good())
        throw std::runtime_error("Failed to open " + fileName);

    auto root = dicomdir.getRootRecord();
    while ((patientRecord = root.nextSub(patientRecord)) != nullptr)
    {
        patientRecord->findAndGetOFString(DCM_PatientName, tmpString);
        patientName = tmpString.c_str();

        while ((studyRecord = patientRecord->nextSub(studyRecord)) != nullptr)
        {
            studyRecord->findAndGetOFString(DCM_StudyID, tmpString);
            PLUGIN_INFO << "Study ID: " << tmpString << std::endl;

            // Read all series and filter according to SeriesInstanceUID
            while ((seriesRecord = studyRecord->nextSub(seriesRecord)) !=
                   nullptr)
            {
                seriesRecord->findAndGetOFString(DCM_SeriesNumber, tmpString);
                PLUGIN_INFO << "Series number: " << tmpString << std::endl;

                size_t nbImages = 0;
                while ((imageRecord = seriesRecord->nextSub(imageRecord)) !=
                       nullptr)
                {
                    OFString refId;
                    imageRecord->findAndGetOFStringArray(DCM_ReferencedFileID,
                                                         refId);

                    // Replace backslashes with slashes
                    std::string str = std::string(refId.data());
                    while (str.find("\\") != std::string::npos)
                        str.replace(str.find("\\"), 1, "/");

                    // Full image filename
                    boost::filesystem::path path = fileName;
                    boost::filesystem::path folder = path.parent_path();
                    const std::string imageFileName =
                        std::string(folder.string()) + "/" + str;

                    // Load image from file
                    DICOMImageDescriptor imageDescriptor;
                    readDICOMFile(imageFileName, imageDescriptor);
                    dicomImages.push_back(imageDescriptor);
                    ++nbImages;
                }
                PLUGIN_DEBUG << nbImages << " images" << std::endl;
                // break; // TODO: Manage multiple series
            }
        }
    }
    return dicomImages;
}

brayns::ModelDescriptorPtr DICOMLoader::readFile(const std::string& fileName)
{
    DICOMImageDescriptor imageDescriptor;
    readDICOMFile(fileName, imageDescriptor);

    // Data range
    const brayns::Vector2f dataRange = {std::numeric_limits<uint16_t>::max(),
                                        std::numeric_limits<uint16_t>::min()};

    auto volume = _scene.createSharedDataVolume(
        {imageDescriptor.dimensions.x(), imageDescriptor.dimensions.y(), 1},
        {imageDescriptor.pixelSpacing.x(), imageDescriptor.pixelSpacing.y(), 1},
        brayns::DataType::UINT16);
    volume->setDataRange(dataRange);
    volume->mapData(imageDescriptor.buffer);

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

    // Data type and range
    brayns::DataType dataType;
    brayns::Vector2f dataRange{std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::min()};

    std::vector<char> volumeData;
    for (const auto& dicomImage : dicomImages)
    {
        volumeData.insert(volumeData.end(), dicomImage.buffer.begin(),
                          dicomImage.buffer.end());
        dataType = dicomImage.dataType;
        dataRange.x() = std::min(dataRange.x(), dicomImage.dataRange.x());
        dataRange.y() = std::max(dataRange.y(), dicomImage.dataRange.y());
    }

    // Create Model
    updateProgress("Creating model ...", 2, 2);
    PLUGIN_INFO << "Creating " << dataTypeToString(dataType) << " volume "
                << dimensions << ", " << elementSpacing << ", " << dataRange
                << " (" << volumeData.size() << " bytes)" << std::endl;
    auto volume =
        _scene.createSharedDataVolume(dimensions, elementSpacing, dataType);
    volume->setDataRange(dataRange);
    volume->mapData(volumeData);
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

brayns::ModelDescriptorPtr DICOMLoader::importFromFolder(
    const std::string& path)
{
    auto files = brayns::parseFolder(path, {".dcm"});
    std::sort(files.begin(), files.end());
    if (files.empty())
        throw std::runtime_error("DICOM folder does not contain any images");

    DICOMImageDescriptors imageDescriptors;
    imageDescriptors.resize(files.size());

    brayns::Vector3ui dimensions;
    brayns::Vector3f elementSpacing = {1, 1, 1};

    brayns::Vector2f dataRange{std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::min()};
    brayns::DataType dataType;

    // Load remaining images
    size_t i = 0;
    for (const auto& file : files)
    {
        auto& id = imageDescriptors[i];
        readDICOMFile(file, id);

        switch (i)
        {
        case 0:
        {
            dataType = id.dataType;
            dimensions = {id.dimensions.x(), id.dimensions.y(), id.nbFrames};
            elementSpacing = {id.pixelSpacing.x(), id.pixelSpacing.y(), 1.f};
            break;
        }
        case 1:
            elementSpacing.z() = abs(imageDescriptors[i].position.z() -
                                     imageDescriptors[i - 1].position.z());
        default:
            dimensions.z() += id.nbFrames;
        }

        dataRange.x() = std::min(dataRange.x(), id.dataRange.x());
        dataRange.y() = std::max(dataRange.y(), id.dataRange.y());

        ++i;
    }

    // Create volume
    PLUGIN_INFO << "Creating " << dataTypeToString(dataType) << " volume "
                << dimensions << ", " << elementSpacing << ", " << dataRange
                << std::endl;

    std::vector<char> volumeData;
    for (const auto& id : imageDescriptors)
        volumeData.insert(volumeData.end(), id.buffer.begin(), id.buffer.end());

    auto volume =
        _scene.createSharedDataVolume(dimensions, elementSpacing, dataType);
    volume->setDataRange(dataRange);
    volume->mapData(volumeData);

    // Create Model
    auto model = _scene.createModel();
    model->addVolume(volume);

    brayns::Transformation transformation;
    transformation.setRotationCenter(model->getBounds().getCenter());
    brayns::ModelMetadata metaData = {{"dimensions", to_string(dimensions)},
                                      {"element-spacing",
                                       to_string(elementSpacing)},
                                      {"data-range", to_string(dataRange)}};
    auto modelDescriptor =
        std::make_shared<brayns::ModelDescriptor>(std::move(model), path,
                                                  metaData);
    modelDescriptor->setTransformation(transformation);
    modelDescriptor->setBoundingBox(true);
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

std::set<std::string> DICOMLoader::getSupportedDataTypes()
{
    return {"dcm", "DICOMDIR"};
}

std::string DICOMLoader::dataTypeToString(const brayns::DataType& dataType)
{
    switch (dataType)
    {
    case brayns::DataType::UINT8:
        return "Unsigned 8bit";
    case brayns::DataType::UINT16:
        return "Unsigned 16bit";
    case brayns::DataType::UINT32:
        return "Unsigned 32bit";
    case brayns::DataType::INT8:
        return "Signed 8bit";
    case brayns::DataType::INT16:
        return "Signed 16bit";
    case brayns::DataType::INT32:
        return "Signed 32bit";
    case brayns::DataType::FLOAT:
        return "Float";
    case brayns::DataType::DOUBLE:
        return "Double";
    }
    return "Undefined";
}
