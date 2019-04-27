using System;
using System.IO;

namespace tempdir
{
    class TemporaryDirectory : IDisposable
    {
        public readonly string Dir;

        public TemporaryDirectory(string basename = "")
        {
            var tempDir = GenerateCandidateDirectoryName(basename);
            Console.WriteLine("Potential temp dir: " + tempDir);
            while (System.IO.Directory.Exists(tempDir))
            {
                Console.WriteLine("Warning: temp dir exists: " + tempDir);
                tempDir = GenerateCandidateDirectoryName(basename);
                Console.WriteLine("Potential temp dir: " + tempDir);
            }
            Console.WriteLine("Creating directory: " + tempDir);
            Directory.CreateDirectory(tempDir);
            if (Directory.Exists(tempDir))
            {
                Console.WriteLine("  Success!");
            }
            else
            {
                Console.WriteLine("  Failure!");
            }
            Dir = tempDir;
        }

        ~TemporaryDirectory()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        private void Dispose(bool itIsSafeToAlsoFreeManagedObjects)
        {
            if (Directory.Exists(Dir))
            {
                Console.WriteLine("Disposing of " + Dir);
                Directory.Delete(Dir, true);
            }
            else
            {
                Console.WriteLine("Error: no directory to dispose: " + Dir);
            }
        }

        private string GenerateCandidateDirectoryName(string basename)
        {
            var tempPath = Path.GetTempPath();
            var randName = Path.GetRandomFileName();
            var randBase = Path.GetFileNameWithoutExtension(randName);
            var tempDir = Path.Combine(tempPath, basename + randBase);
            return tempDir;
        }
    }

    class Program
    {
        static void Main(string[] args)
        {
            for (int i = 0; i < 10; i++)
            {
                //string fname = Path.GetRandomFileName();
                //string ext = Path.GetExtension(fname);
                //string changed = Path.ChangeExtension(fname, "");
                //string basename = Path.GetFileNameWithoutExtension(fname);
                //Console.WriteLine("Random file name:  " + fname);
                //Console.WriteLine("  ext:             " + ext);
                //Console.WriteLine("  changed:         " + changed);
                //Console.WriteLine("  without ext:     " + basename);
                //Console.WriteLine();
                
                using (var tmpdir = new TemporaryDirectory("tempdir-"))
                {
                    Console.WriteLine("Created temporary directory: "
                                      + tmpdir.Dir);
                    var newfile = Path.Combine(tmpdir.Dir, "newfile.txt");
                    File.WriteAllLines(newfile, new string[] {});
                    if (File.Exists(newfile))
                    {
                        Console.WriteLine("Created: " + newfile);
                    }
                    else
                    {
                        Console.WriteLine("Failed to create " + newfile);
                    }
                }
                Console.WriteLine();
            }
        }
    }
}
