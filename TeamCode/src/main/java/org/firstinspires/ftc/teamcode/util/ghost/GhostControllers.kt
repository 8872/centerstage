package org.firstinspires.ftc.teamcode.util.ghost

class GhostControllers(vararg var controllers: GhostController) {

    fun update() {
        controllers.forEach { it.update() }
    }

    operator fun get(index: Int): GhostController {
        return controllers[index]
    }
}