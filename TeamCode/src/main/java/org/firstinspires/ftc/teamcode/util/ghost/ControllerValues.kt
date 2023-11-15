package org.firstinspires.ftc.teamcode.util.ghost

open class ControllerValues<T>(
        defaultVal: T,
        vals: ArrayList<T>,
        syms: ArrayList<String>
) {
    private var defaultValue: T = defaultVal
    private var values: ArrayList<T> = ArrayList(vals)
    private var previousValues: ArrayList<T> = ArrayList(vals)
    private var symbols: ArrayList<String> = ArrayList(syms)

    fun setValue(symbol: String, value: T) {
        for (i in symbols.indices) {
            if (symbol == symbols[i]) {
                values[i] = value
            }
        }
    }

    fun getValue(symbol: String): T {
        var valResult: T = defaultValue
        for (i in symbols.indices) {
            if (symbol == symbols[i]) {
                valResult = values[i]
            }
        }
        previousValues = ArrayList(values)
        return valResult
    }

    fun getSymbolsOfChanged(): ArrayList<String> {
        val changedSymbols = ArrayList<String>()
        for (i in values.indices) {
            if (previousValues[i] != values[i]) {
                changedSymbols.add(symbols[i])
            }
        }
        return changedSymbols
    }

    fun reset() {
        for (i in values.indices) {
            values[i] = defaultValue
        }
    }
}