#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkMetaDataObject.h>
#include <itkNrrdImageIO.h>

#include <chrono>

using itk::operator<<;

int main(int arg_count, char *arg_list[]) {
  if (arg_count < 2) {
    std::cerr << "Usage: " << arg_list[0] << " file-in" << std::endl;
    return 1;
  }

  using PixelType = unsigned char;
  constexpr int Dims = 3;
  using ImageType = itk::Image<PixelType, Dims>;
  using ReaderType = itk::ImageFileReader<ImageType>;

  // read the file
  auto reader = ReaderType::New();
  reader->SetFileName(arg_list[1]);
  reader->Update();

  auto image = reader->GetOutput();
  std::cout
    << "Image:               " << *image << "\n"
    << "Origin:              " << image->GetOrigin() << "\n"
    << "Direction:         \n" << image->GetDirection()
    << "Inverse Direction: \n" << image->GetInverseDirection()
    << "Spacing:             " << image->GetSpacing() << "\n"
    << "Largest Region:      " << image->GetLargestPossibleRegion() << "\n"
    << "Buffered Region:     " << image->GetBufferedRegion() << "\n"
    << "Requested Region:    " << image->GetRequestedRegion() << "\n"
    << "Offset Table:        " << image->GetOffsetTable() << "\n"
    //<< "Data Released?:      " << image->GetDataReleased() << "\n"
    //<< "Release Data Flag:   " << image->GetReleaseDataFlag() << "\n"
    //<< "Source:              " << image->GetSource() << "\n"
    //<< "Source Output Index: " << image->GetSourceOutputIndex() << "\n"
    //<< "Source Output Name:  " << image->GetSourceOutputName() << "\n"
    //<< "Pipeline MTime:      " << image->GetPipelineMTime() << "\n"
    //<< "Update MTime:        " << image->GetUpdateMTime() << "\n"
    //<< "Real Time Stamp:     " << image->GetRealTimeStamp() << "\n"
    //<< "Meta Data Dict:      " << image->GetMetaDataDictionary() << "\n"
    << std::endl;

  return 0;
}
