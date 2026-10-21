
const fs = require('fs');
const path = require('path');

const inputPath = path.resolve(
    __dirname, '..', '..', 'tools', 'odrive', 'enums.py'
);

const outputPath = path.resolve(
    __dirname, '..', 'src', 'assets', 'odriveEnums.json'
);

fs.readFile(inputPath, 'utf8', (err, data) => {
    if (err) throw err;

    const enums = {};

    // Handle Windows, Linux and older Mac line endings.
    const lines = data.split(/\r\n|\n|\r/);

    for (const line of lines) {
        const trimmed = line.trim();

        // Ignore comments and empty lines.
        if (!trimmed || trimmed.startsWith('#')) {
            continue;
        }

        // Match numeric constant assignments, including hexadecimal.
        const match = trimmed.match(
            /^([A-Z][A-Z0-9_]*)\s*=\s*(0[xX][0-9a-fA-F]+|\d+)\s*(?:#.*)?$/
        );

        if (!match) {
            continue;
        }

        const name = match[1];
        const value = Number(match[2]);

        enums[name] = value;
    }

    fs.writeFile(
        outputPath,
        JSON.stringify(enums, null, 4) + '\n',
        'utf8',
        err => {
            if (err) throw err;

            console.log(
                `Generated ${Object.keys(enums).length} enums`
            );
            console.log(`Output: ${outputPath}`);
        }
    );
});
