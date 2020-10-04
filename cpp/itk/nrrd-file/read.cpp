#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkMetaDataObject.h>
#include <itkNrrdImageIO.h>

#include <chrono>

template <typename Func>
float time_function_call(Func f) {
  auto before = std::chrono::steady_clock::now();
  f();
  auto after = std::chrono::steady_clock::now();
  auto nanosec = std::chrono::duration_cast<
      std::chrono::nanoseconds>(after - before).count();
  return nanosec / 1.0e9f; // convert to seconds as a float
}

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

  auto io = itk::NrrdImageIO::New();

  using WriterType = itk::ImageFileWriter<ImageType>;
  for (int compression_level = 0; compression_level < 10; compression_level++) {
    auto writer = WriterType::New();
    writer->UseInputMetaDataDictionaryOn();
    writer->SetInput(reader->GetOutput());
    writer->SetImageIO(io);
    auto fname = "tmp-" + std::to_string(compression_level) + ".nrrd";
    writer->SetFileName(fname);
    writer->UseCompressionOn();
    writer->SetCompressionLevel(compression_level);
    std::cout << "writing " << fname << std::flush;
    auto secs = time_function_call([&writer]() { writer->Update(); });
    std::cout << " (" << secs << " seconds)" << std::endl;
  }

  return 0;
}
