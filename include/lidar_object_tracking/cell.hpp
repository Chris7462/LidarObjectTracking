#pragma once

class Cell
{
  public:
    Cell();
    ~Cell() = default;
    void update_min_z(float z);
    void update_height(float h);
    void update_smoothed(float s);
    void update_hdiff(float hd);
    void update_ground();
    bool is_this_ground();
    float get_min_z();
    float get_height();
    float get_hdiff();
    float get_smoothed();
    float get_hground();

  private:
    float smoothed;
    float height;
    float hdiff;
    float hground;
    float min_z;
    bool is_ground;
};
