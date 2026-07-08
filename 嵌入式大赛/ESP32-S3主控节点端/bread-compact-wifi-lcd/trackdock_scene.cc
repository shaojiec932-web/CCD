#ifdef TRACKDOCK_SCENE_IMPLEMENTATION
#include "trackdock_scene.h"

#include <cstring>

static const char* current_scene_id = "default";

struct SceneInfo {
    const char* id;
    const char* name;
};

static const SceneInfo SCENES[] = {
    {"factory", "工厂"},
    {"hospital", "医院"},
    {"eldercare", "养老院"},
    {"library", "图书馆"},
    {"teaching", "教学楼"},
    {"default", "默认"},
};

static const SceneInfo* FindScene(const char* scene_id) {
    if (!scene_id || scene_id[0] == '\0') return &SCENES[5];
    for (const auto& scene : SCENES) {
        if (strcmp(scene.id, scene_id) == 0) {
            return &scene;
        }
    }
    return &SCENES[5];
}

void TrackDockScene_Set(const char* scene_id) {
    current_scene_id = FindScene(scene_id)->id;
}

const char* TrackDockScene_GetId() {
    return current_scene_id;
}

const char* TrackDockScene_GetName() {
    return FindScene(current_scene_id)->name;
}
#endif // TRACKDOCK_SCENE_IMPLEMENTATION
