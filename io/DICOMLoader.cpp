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

#include <brayns/common/utils/utils.h>
#include <brayns/engineapi/Model.h>
#include <brayns/engineapi/Scene.h>
#include <brayns/engineapi/SharedDataVolume.h>

#include <dcmtk/dcmdata/dcddirif.h>
#include <dcmtk/dcmdata/dctk.h>
#include <dcmtk/dcmimgle/dcmimage.h>

#include <boost/filesystem.hpp>

namespace
{
const std::string SUPPORTED_BASENAME_DICOMDIR = "DICOMDIR";
const std::string SUPPORTED_EXTENSION_DCM = "dcm";
const brayns::ColorMap colormap = {
    "DICOM",
    {{0.21960784494876862, 0.0, 0.0},
     {0.43921568989753723, 0.0, 0.0},
     {0.6666666865348816, 0.16470588743686676, 0.0},
     {0.886274516582489, 0.3843137323856354, 0.0},
     {1.0, 0.6117647290229797, 0.11372549086809158},
     {1.0, 0.8313725590705872, 0.3294117748737335},
     {1.0, 1.0, 0.5607843399047852},
     {1.0, 1.0, 0.7764706015586853}}};
const brayns::Vector2ds controlPoints = {{0.0, 0.0},  {0.125, 0.0},
                                         {0.25, 0.0}, {0.375, 0.0},
                                         {0.5, 1.0},  {0.625, 1.0},
                                         {0.75, 1.0}, {0.875, 1.0},
                                         {1.0, 1.0}};

} // namespace

DICOMLoader::DICOMLoader(brayns::Scene& scene,
                         const brayns::GeometryParameters& geometryParameters,
                         brayns::PropertyMap&& loaderParams)
    : Loader(scene)
    , _geometryParameters(geometryParameters)
    , _loaderParams(loaderParams)
{
}

void DICOMLoader::_setDefaultTransferFunction(
    brayns::Model& model, const brayns::Vector2f& dataRange) const
{
    auto& tf = model.getTransferFunction();
    tf.setValuesRange(dataRange);
    tf.setColorMap(colormap);
    tf.setControlPoints(controlPoints);
}

void DICOMLoader::_readDICOMFile(const std::string& fileName,
                                 DICOMImageDescriptor& imageDescriptor) const
{
    DcmFileFormat file;
    file.loadFile(fileName.c_str());
    DcmDataset* dataset = file.getDataset();
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

DICOMImageDescriptors DICOMLoader::_parseDICOMImagesData(
    const std::string& fileName, brayns::ModelMetadata& metadata) const
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
        patientRecord->findAndGetOFString(DCM_PatientID, tmpString);
        metadata["Patient ID"] = tmpString.c_str();
        patientRecord->findAndGetOFString(DCM_PatientName, tmpString);
        metadata["Patient name"] = tmpString.c_str();
        patientRecord->findAndGetOFString(DCM_PatientAge, tmpString);
        metadata["Patient age"] = tmpString.c_str();
        patientRecord->findAndGetOFString(DCM_PatientSex, tmpString);
        metadata["Patient sex"] = tmpString.c_str();
        patientRecord->findAndGetOFString(DCM_PatientBirthDate, tmpString);
        metadata["Patient date of birth"] = tmpString.c_str();

        while ((studyRecord = patientRecord->nextSub(studyRecord)) != nullptr)
        {
            studyRecord->findAndGetOFString(DCM_StudyID, tmpString);
            metadata["Study ID"] = tmpString.c_str();

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
                    _readDICOMFile(imageFileName, imageDescriptor);
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

brayns::ModelDescriptorPtr DICOMLoader::_readFile(
    const std::string& fileName) const
{
    DICOMImageDescriptor imageDescriptor;
    _readDICOMFile(fileName, imageDescriptor);

    // Data range
    const brayns::Vector2f dataRange = {std::numeric_limits<uint16_t>::max(),
                                        std::numeric_limits<uint16_t>::min()};

    // Create Model
    auto model = _scene.createModel();
    if (!model)
        throw std::runtime_error("Failed to create model");

    auto volume = model->createSharedDataVolume(
        {imageDescriptor.dimensions.x, imageDescriptor.dimensions.y, 1},
        {imageDescriptor.pixelSpacing.x, imageDescriptor.pixelSpacing.y, 1},
        brayns::DataType::UINT16);
    if (!volume)
        throw std::runtime_error("Failed to create volume");

    volume->setDataRange(dataRange);
    volume->mapData(imageDescriptor.buffer);
    model->addVolume(volume);

    // Transfer function initialization
    _setDefaultTransferFunction(*model, dataRange);

    // Transformation
    brayns::Transformation transformation;
    transformation.setRotationCenter(model->getBounds().getCenter());
    brayns::ModelMetadata metaData = {
        {"Dimensions", to_string(imageDescriptor.dimensions)},
        {"Element spacing", to_string(imageDescriptor.pixelSpacing)}};

    auto modelDescriptor =
        std::make_shared<brayns::ModelDescriptor>(std::move(model), fileName,
                                                  metaData);
    modelDescriptor->setTransformation(transformation);
    return modelDescriptor;
}

brayns::ModelDescriptorPtr DICOMLoader::_readDirectory(
    const std::string& fileName, const brayns::LoaderProgress& callback) const
{
    brayns::ModelMetadata metaData;
    const auto& dicomImages = _parseDICOMImagesData(fileName, metaData);

    if (dicomImages.empty())
        throw std::runtime_error("DICOM folder does not contain any images");

    // Dimensions
    brayns::Vector3ui dimensions = {dicomImages[0].dimensions.x,
                                    dicomImages[0].dimensions.y,
                                    (unsigned int)dicomImages.size()};

    // Element spacing (if single image, assume that z pixel spacing is the
    // same as y
    brayns::Vector3f elementSpacing{dicomImages[0].pixelSpacing.x,
                                    dicomImages[0].pixelSpacing.y,
                                    dicomImages[0].pixelSpacing.y};
    if (dicomImages.size() > 1)
        elementSpacing.z =
            dicomImages[1].position.z - dicomImages[0].position.z;

    // Load images into volume
    callback.updateProgress("Loading voxels ...", 0.5f);

    // Data type and range
    brayns::DataType dataType;
    brayns::Vector2f dataRange{std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::min()};

    uint8_ts volumeData;
    for (const auto& dicomImage : dicomImages)
    {
        volumeData.insert(volumeData.end(), dicomImage.buffer.begin(),
                          dicomImage.buffer.end());
        dataType = dicomImage.dataType;
        dataRange.x = std::min(dataRange.x, dicomImage.dataRange.x);
        dataRange.y = std::max(dataRange.y, dicomImage.dataRange.y);
    }

    // Create Model
    callback.updateProgress("Creating model ...", 1.f);
    PLUGIN_INFO << "Creating " << _dataTypeToString(dataType) << " volume "
                << dimensions << ", " << elementSpacing << ", " << dataRange
                << " (" << volumeData.size() << " bytes)" << std::endl;
    auto model = _scene.createModel();
    if (!model)
        throw std::runtime_error("Failed to create model");

    auto volume =
        model->createSharedDataVolume(dimensions, elementSpacing, dataType);
    if (!volume)
        throw std::runtime_error("Failed to create volume");

    volume->setDataRange(dataRange);
    volume->mapData(volumeData);
    model->addVolume(volume);

    // Transfer function initialization
    _setDefaultTransferFunction(*model, dataRange);

    // Transformation
    brayns::Transformation transformation;
    transformation.setRotationCenter(model->getBounds().getCenter());
    metaData["Dimensions"] = to_string(dimensions);
    metaData["Element spacing"] = to_string(elementSpacing);

    auto modelDescriptor =
        std::make_shared<brayns::ModelDescriptor>(std::move(model), "DICOMDIR",
                                                  metaData);
    modelDescriptor->setTransformation(transformation);
    return modelDescriptor;
}

brayns::ModelDescriptorPtr DICOMLoader::importFromFolder(
    const std::string& path)
{
    auto files = brayns::parseFolder(path, {"." + SUPPORTED_EXTENSION_DCM});
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
        _readDICOMFile(file, id);

        switch (i)
        {
        case 0:
        {
            dataType = id.dataType;
            dimensions = {id.dimensions.x, id.dimensions.y, id.nbFrames};
            elementSpacing = {id.pixelSpacing.x, id.pixelSpacing.y, 1.f};
            break;
        }
        case 1:
            elementSpacing.z = abs(imageDescriptors[i].position.z -
                                   imageDescriptors[i - 1].position.z);
            break;
        default:
            dimensions.z += id.nbFrames;
        }

        dataRange.x = std::min(dataRange.x, id.dataRange.x);
        dataRange.y = std::max(dataRange.y, id.dataRange.y);

        ++i;
    }

    // Create volume
    PLUGIN_INFO << "Creating " << _dataTypeToString(dataType) << " volume "
                << dimensions << ", " << elementSpacing << ", " << dataRange
                << std::endl;

    uint8_ts volumeData;
    for (const auto& id : imageDescriptors)
        volumeData.insert(volumeData.end(), id.buffer.begin(), id.buffer.end());

    // Create Model
    auto model = _scene.createModel();
    if (!model)
        throw std::runtime_error("Failed to create model");

    auto volume =
        model->createSharedDataVolume(dimensions, elementSpacing, dataType);
    if (!volume)
        throw std::runtime_error("Failed to create volume");

    volume->setDataRange(dataRange);
    volume->mapData(volumeData);
    model->addVolume(volume);

    brayns::Transformation transformation;
    transformation.setRotationCenter(model->getBounds().getCenter());
    brayns::ModelMetadata metaData = {{"Dimensions", to_string(dimensions)},
                                      {"Element spacing",
                                       to_string(elementSpacing)},
                                      {"Data range", to_string(dataRange)}};
    auto modelDescriptor =
        std::make_shared<brayns::ModelDescriptor>(std::move(model), path,
                                                  metaData);
    modelDescriptor->setTransformation(transformation);
    modelDescriptor->setBoundingBox(true);
    return modelDescriptor;
}

brayns::ModelDescriptorPtr DICOMLoader::importFromFile(
    const std::string& path, const brayns::LoaderProgress& callback,
    const brayns::PropertyMap& /*properties*/) const
{
    PLUGIN_INFO << "Importing DICOM dataset from " << path << std::endl;
    const auto extension = boost::filesystem::extension(path);
    if (extension == "." + SUPPORTED_EXTENSION_DCM)
        return _readFile(path);
    return _readDirectory(path, callback);
}

brayns::ModelDescriptorPtr DICOMLoader::importFromBlob(
    brayns::Blob&&, const brayns::LoaderProgress&,
    const brayns::PropertyMap&) const
{
    throw std::runtime_error("Loading DICOM from blob is not supported");
}

std::string DICOMLoader::getName() const
{
    return "Loader for DICOM datasets";
}

std::vector<std::string> DICOMLoader::getSupportedExtensions() const
{
    return {SUPPORTED_EXTENSION_DCM, SUPPORTED_BASENAME_DICOMDIR};
}

bool DICOMLoader::isSupported(const std::string& filename,
                              const std::string& extension) const
{
    const auto basename = boost::filesystem::basename(filename);
    const std::set<std::string> basenames = {SUPPORTED_BASENAME_DICOMDIR};
    const std::set<std::string> extensions = {SUPPORTED_EXTENSION_DCM};
    return (basenames.find(basename) != basenames.end() ||
            extensions.find(extension) != extensions.end());
}

std::string DICOMLoader::_dataTypeToString(
    const brayns::DataType& dataType) const
{
    switch (dataType)
    {
    case brayns::DataType::UINT8:
        return "Unsigned 8bit";
    case brayns::DataType::UINT16:
        return "Unsigned 16bit ";
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

brayns::PropertyMap DICOMLoader::getCLIProperties()
{
    brayns::PropertyMap pm("DICOMLoader");
    return pm;
}
