#pragma once
#include <core/types.h>
#include <vector>
#include <memory>
#include <functional>

class EngineContext;
class ImGuiPass;

class IRenderPass
{
public:
    virtual ~IRenderPass() = default;

    virtual void init(EngineContext *context) = 0;

    virtual void cleanup() = 0;

    virtual void execute(VkCommandBuffer cmd) = 0;

    virtual const char *getName() const = 0;
};

class RenderPassManager
{
public:
    void init(EngineContext *context);

    void cleanup();

    void addPass(std::unique_ptr<IRenderPass> pass);

    void setImGuiPass(std::unique_ptr<IRenderPass> imguiPass);

    ImGuiPass *getImGuiPass();

    template<typename T>
    T *getPass()
    {
        for (auto &pass: _passes)
        {
            if (T *typedPass = dynamic_cast<T *>(pass.get()))
            {
                return typedPass;
            }
        }
        return nullptr;
    }

private:
    EngineContext *_context = nullptr;
    std::vector<std::unique_ptr<IRenderPass> > _passes;
    std::unique_ptr<IRenderPass> _imguiPass = nullptr;
};
