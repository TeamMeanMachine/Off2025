package frc.team2471.off2025.util.quix

class AlignmentState {
    enum class ReefStackChoice {
        LEFT,
        RIGHT
    }

    enum class ReefLevel {
        ONE,
        TWO,
        THREE,
        FOUR
    }

    var stackChoice: ReefStackChoice? = ReefStackChoice.RIGHT
    var scoringLevel: ReefLevel? = ReefLevel.ONE

    companion object {
        private var m_instance: AlignmentState? = null

        val instance: AlignmentState
            get() {
                if (m_instance == null) {
                    m_instance = AlignmentState()
                }
                return m_instance!!
            }
    }
}
