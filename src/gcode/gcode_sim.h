#pragma once
#include <vector>
#include <string>
#include "merlin.h"
#include <glm/glm.hpp>

struct Command {
    glm::vec3 position;
    float speed;
    float flow;
    int material;
};

class GCodeSimulator{
public:
    GCodeSimulator();
    void readFile(const std::string& filepath);
    void readGCode(const std::string& gcode);
    void partGCodeLine(const std::string& line, Command& command);

    inline void move(const glm::vec3& position, float flow, float speed = 20) {

        m_commands.push_back({position + glm::vec3(0,0,0.08*10.0), speed, flow, m_current_material});
        //Merlin::Console::print() << position << Merlin::Console::endl;
        m_commands_str.push_back("CUSTOM_COMMAND"); // Placeholder for string representation
    }

    inline void setMaterial(int mat) {
        m_current_material = mat;
    }


    void reset();
    void update(float dt);

    void setNozzlePosition(const glm::vec3& pos);
    glm::vec3 getNozzlePosition();
    glm::vec3 getNozzleVelocity();
    int getNozzleMaterial();
    float getFlowrate();
	bool lastCommandReached();
    void clear();
	inline int index() const { return currentIndex; }
private:
    std::vector<Command> m_commands;
    std::vector<std::string> m_commands_str;
    size_t currentIndex = 0;

    float m_current_speed = 0;
    float m_current_flowrate = 0;
	int m_current_material = 0;
	
    glm::vec3 m_current_position = glm::vec3(0);
    glm::vec3 m_current_velocity = glm::vec3(0);
    glm::vec3 m_current_target = glm::vec3(0);
    glm::vec3 m_origin_offset = glm::vec3(-126,-126, 0.26 );
};

