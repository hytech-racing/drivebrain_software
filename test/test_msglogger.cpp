#include <MsgLogger.hpp>
#include <fstream>      // std::ofstream

class IntegerFileWriter
{
public:
    // Constructor (optional file name can be set here, but now handled in openFile)
    IntegerFileWriter() {}

    // Function to open the file with a given filename
    void openFile(const std::string &filename)
    {
        filename_ = filename;
        file_.open(filename_, std::ios::out | std::ios::trunc); // Open the file in output mode (truncate by default)
        if (!file_.is_open())
        {
            std::cerr << "Unable to open file: " << filename_ << std::endl;
        }
        else
        {
            std::cout << "File opened: " << filename_ << std::endl;
        }
    }

    // Function to write an integer to the file
    void writeInteger(int value)
    {
        if (file_.is_open())
        {
            file_ << value << "\n";
            // std::cout << "Integer written to file: " << value << std::endl;
        }
        else
        {
            std::cerr << "File is not open. Cannot write integer." << std::endl;
        }
    }

    // Function to close the file
    void closeFile()
    {
        if (file_.is_open())
        {
            file_.close();
            std::cout << "File closed." << std::endl;
        }
        else
        {
            std::cerr << "File is not open. Cannot close." << std::endl;
        }
    }

    // Destructor to ensure the file is closed when the object is destroyed
    ~IntegerFileWriter()
    {
        if (file_.is_open())
        {
            file_.close();
        }
    }

private:
    std::string filename_;
    std::ofstream file_; // File stream member to manage the file state
};

int main()
{

    std::function<void(int)> live_telem_func = [&](int n) { std::cout << "live out " << n <<std::endl; };
    IntegerFileWriter fw;
    core::MsgLogger<int> logger_test(std::string("."), std::string(".txt"), true,
                               std::bind(&IntegerFileWriter::writeInteger, std::ref(fw), std::placeholders::_1),
                               std::bind(&IntegerFileWriter::closeFile, std::ref(fw)),
                               std::bind(&IntegerFileWriter::openFile, std::ref(fw), std::placeholders::_1),
                               live_telem_func);
    
    int out =0;
    while(out < 100)
    {
        out++;
        logger_test.log_msg(out);
    }

    while(logger_test.messages_still_need_output())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    std::cout << "done logging, stopping" <<std::endl; 
    logger_test.stop_logging_to_file();
}