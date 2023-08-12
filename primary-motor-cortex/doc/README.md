# Documentation

Welcome to the documentation folder of the Roboost Primary Motor Cortex.

This folder contains documentation related to the components's codebase. The documentation is generated using Doxygen, a powerful tool for automatically generating documentation from source code comments.

## Generating Documentation

To generate the documentation for this project, follow these steps:

1. **Install Doxygen:**
   If you haven't already, make sure you have Doxygen installed on your system. You can download and install it from the official Doxygen website (<https://www.doxygen.nl/download.html>) or using your system's package manager.

2. **Configure Doxygen:**
   The configuration for generating documentation is defined in the `Doxyfile` located in the project's root folder. You can adjust the settings in this file to tailor the documentation to your needs.

3. **Generate Documentation:**
   Open a terminal and navigate to the project's root folder.

    ```bash
    cd path/to/your/project#
    ```

    Run Doxygen with the Doxyfile as the input configuration file.

    ```bash
    doxygen Doxyfile
    ```

   Doxygen will process your source code comments and generate the documentation in the specified output directory (default: `./docs`).

4. **View Documentation:**
Once the documentation is generated, you can open the HTML files in a web browser to view the documentation. The main entry point is usually `index.html`.

## Documentation Output

The generated documentation is stored in the `doc` folder at the root of the project. You can find various HTML files, diagrams, and other resources related to the project's documentation in this folder.

Feel free to explore the generated documentation to understand the project's structure, classes, methods, and their relationships.

## Customizing Documentation

You can customize the documentation appearance, structure, and content by modifying the `Doxyfile` settings. Doxygen provides a wide range of options to tailor the documentation to your preferences.

For more information on Doxygen settings and customization, refer to the official Doxygen documentation (<https://www.doxygen.nl/manual/index.html>).

---

For more information about the project itself, please refer to the project's main README.md file.

If you encounter any issues or have questions regarding the documentation, feel free to open an issue on the project's GitHub repository.
