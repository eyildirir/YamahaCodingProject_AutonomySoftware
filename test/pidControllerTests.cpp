#include "../include/pidController.hpp"
#include <tuple>
#include <iostream>

namespace PIDTests
{
    /**
     * @brief Test function for testing the getters and setters.
     *
     * @return true All tests passed.
     * @return false One or more tests failed.
     */
    bool getter_setter_test()
    {
        std::cout << "Controller Getter/Setter Utility Tests:" << std::endl;
        bool result = true;
        PIDController subject;
        std::tuple<double, double, double> coeffs;
        std::tuple<double, double, double> coeffs2;

        double kp1, kd1, ki1, kp2, kd2, ki2;
        uint8_t test_bits;

        /**
         * @brief Getters parity test
         *
         */
        coeffs = subject.get_coeffs();
        std::tie(kp1, kd1, ki1) = coeffs;

        std::cout << "    Testing parity between getters..." << std::endl;

        kp2 = subject.get_Kp();
        kd2 = subject.get_Kd();
        ki2 = subject.get_Ki();

        test_bits = uint8_t(kp1 == kp2) + 2 * uint8_t(kd1 == kd2) + 4 * uint8_t(ki1 == ki2);

        switch (test_bits)
        {
        case 0:
            std::cout << "        All coeffs MISMATCH!" << std::endl;
            result = false;
            break;
        case 1:
            std::cout << "        Kd and Ki MISMATCH!" << std::endl;
            result = false;
            break;
        case 2:
            std::cout << "        Kp and Ki MISMATCH!" << std::endl;
            result = false;
            break;
        case 3:
            std::cout << "        Ki MISMATCH!" << std::endl;
            result = false;
            break;
        case 4:
            std::cout << "        Kp and Kd MISMATCH!" << std::endl;
            result = false;
            break;
        case 5:
            std::cout << "        Kd MISMATCH!" << std::endl;
            result = false;
            break;
        case 6:
            std::cout << "        Kp coeffs MISMATCH!" << std::endl;
            result = false;
            break;
        case 7:
            std::cout << "        All coeffs MATCH!" << std::endl;
            break;
        default:
            std::cout << "        === UNKNOWN ERROR! ===" << std::endl;
            result = false;
            break;
        }

        std::cout << std::endl;

        /**
         * @brief set_coeffs(double kp, double kd, double ki) test
         *
         */
        coeffs = subject.get_coeffs();
        std::tie(kp1, kd1, ki1) = coeffs;

        std::cout << "    Testing \"set_coeffs(double kp, double kd, double ki)\"..." << std::endl;
        subject.set_coeffs(++kp1, ++kd1, ++ki1);

        coeffs2 = subject.get_coeffs();
        std::tie(kp2, kd2, ki2) = coeffs2;

        test_bits = uint8_t(kp1 == kp2) + 2 * uint8_t(kd1 == kd2) + 4 * uint8_t(ki1 == ki2);

        switch (test_bits)
        {
        case 0:
            std::cout << "        All coeffs FAULTY!" << std::endl;
            result = false;
            break;
        case 1:
            std::cout << "        Kd and Ki FAULTY!" << std::endl;
            result = false;
            break;
        case 2:
            std::cout << "        Kp and Ki FAULTY!" << std::endl;
            result = false;
            break;
        case 3:
            std::cout << "        Ki FAULTY!" << std::endl;
            result = false;
            break;
        case 4:
            std::cout << "        Kp and Kd FAULTY!" << std::endl;
            result = false;
            break;
        case 5:
            std::cout << "        Kd FAULTY!" << std::endl;
            result = false;
            break;
        case 6:
            std::cout << "        Kp coeffs FAULTY!" << std::endl;
            result = false;
            break;
        case 7:
            std::cout << "        All coeffs CORRECT!" << std::endl;
            break;
        default:
            std::cout << "        === UNKNOWN ERROR! ===" << std::endl;
            result = false;
            break;
        }

        std::cout << std::endl;

        /**
         * @brief set_coeffs(std::tuple<double, double, double> coeffs) test
         *
         */
        coeffs = subject.get_coeffs();
        std::tie(kp1, kd1, ki1) = coeffs;

        std::cout << "    Testing \"set_coeffs(std::tuple<double, double, double> coeffs)\"..." << std::endl;
        subject.set_coeffs(std::make_tuple(++kp1, ++kd1, ++ki1));

        coeffs2 = subject.get_coeffs();
        std::tie(kp2, kd2, ki2) = coeffs2;

        test_bits = uint8_t(kp1 == kp2) + 2 * uint8_t(kd1 == kd2) + 4 * uint8_t(ki1 == ki2);

        switch (test_bits)
        {
        case 0:
            std::cout << "        All coeffs FAULTY!" << std::endl;
            result = false;
            break;
        case 1:
            std::cout << "        Kd and Ki FAULTY!" << std::endl;
            result = false;
            break;
        case 2:
            std::cout << "        Kp and Ki FAULTY!" << std::endl;
            result = false;
            break;
        case 3:
            std::cout << "        Ki FAULTY!" << std::endl;
            result = false;
            break;
        case 4:
            std::cout << "        Kp and Kd FAULTY!" << std::endl;
            result = false;
            break;
        case 5:
            std::cout << "        Kd FAULTY!" << std::endl;
            result = false;
            break;
        case 6:
            std::cout << "        Kp coeffs FAULTY!" << std::endl;
            result = false;
            break;
        case 7:
            std::cout << "        All coeffs CORRECT!" << std::endl;
            break;
        default:
            std::cout << "        === UNKNOWN ERROR! ===" << std::endl;
            result = false;
            break;
        }

        std::cout << std::endl;

        // Return overall result for potential use in other applications.
        return result;
    }

    /**
     * @brief Unary input response tests to validate the behavior of the PID Controller. 
     * 
     * @param dt Time step.
     * @return true All tests passed.
     * @return false One or more tests failed.
     */
    bool response_test(double dt)
    {
        std::cout << "Controller Response Tests:" << std::endl;
        bool result = true;
        double err = 1;
        double cv;

        /**
         * @brief Proportional response to unary error.
         *
         */
        std::cout << "    Testing proportional response to unary input..." << std::endl;
        PIDController prop(1, 0, 0, dt);
        for (int i = 0; i * 0.01 <= 10; i++)
        {
            cv = prop(err);
        }
        std::cout << "        " << cv << std::endl;

        if (fabs(cv - 1) < 0.01)
        {
            std::cout << "        Expected response; SUCCESS!" << std::endl;
        }
        else
        {
            std::cout << "        Expected response with an error of" << cv - 1 << "; FAILURE!" << std::endl;
            result = false;
        }

        std::cout << std::endl;

        /**
         * @brief Differential response to unary error.
         *
         */
        std::cout << "    Testing differential response to unary input..." << std::endl;
        PIDController diff(0, 1, 0,dt);
        for (int i = 0; i * dt <= 10; i++)
        {
            cv = diff(err);
        }
        std::cout << "        " << cv << std::endl;

        if (fabs(cv - 0.5/dt) < 0.01)
        {
            std::cout << "        Expected response; SUCCESS!" << std::endl;
        }
        else
        {
            std::cout << "        Expected response with an error of " << cv - 0 << "; FAILURE!" << std::endl;
            result = false;
        }

        std::cout << std::endl;

        /**
         * @brief Integral response to unary error.
         *
         */
        std::cout << "    Testing integral response to unary input..." << std::endl;
        PIDController integ(0, 0, 1, dt);
        for (int i = 0; i * dt <= 10; i++)
        {
            cv = integ(err);
        }
        std::cout << "        " << cv << std::endl;

        if (fabs(cv - 10) < dt)
        {
            std::cout << "        Expected response; SUCCESS!" << std::endl;
        }
        else
        {
            std::cout << "        Expected response with an error of " << cv - 10 << "; FAILURE!" << std::endl;
            result = false;
        }

        std::cout << std::endl;

        // Return overall result for potential use in other applications.
        return result;
    }
}

int main()
{
    PIDTests::getter_setter_test();
    PIDTests::response_test(0.01);
    return 0;
}