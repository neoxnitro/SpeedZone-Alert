# Contributing to SpeedZone-Alert

Thank you for your interest in contributing to SpeedZone-Alert! This document provides guidelines for contributing to the project.

## Ways to Contribute

### 1. Report Bugs
- Check existing issues first to avoid duplicates
- Include detailed description of the problem
- Provide serial monitor output
- Include hardware setup photos if relevant
- Specify ESP32 and GPS module models

### 2. Suggest Features
- Open an issue with [Feature Request] in the title
- Describe the feature and its benefits
- Explain use cases
- Consider backward compatibility

### 3. Improve Documentation
- Fix typos or unclear explanations
- Add missing information
- Improve examples
- Translate to other languages

### 4. Add Speed Camera Databases
- Create regional camera database files
- Verify accuracy of locations
- Include source references
- Format consistently

### 5. Code Contributions
- Fix bugs
- Implement new features
- Optimize performance
- Improve code quality

## Getting Started

### 1. Fork the Repository
```bash
# Fork on GitHub, then clone your fork
git clone https://github.com/YOUR_USERNAME/SpeedZone-Alert.git
cd SpeedZone-Alert
```

### 2. Create a Branch
```bash
git checkout -b feature/your-feature-name
# or
git checkout -b fix/bug-description
```

### 3. Make Changes
- Follow existing code style
- Test thoroughly on hardware
- Document your changes
- Add comments where needed

### 4. Test Your Changes
- Upload to ESP32
- Test with actual hardware
- Verify GPS functionality
- Test all alert scenarios
- Check serial monitor output

### 5. Commit Your Changes
```bash
git add .
git commit -m "Brief description of changes"
```

Use clear commit messages:
- `Add: New feature description`
- `Fix: Bug description`
- `Docs: Documentation changes`
- `Refactor: Code improvement`

### 6. Push and Create Pull Request
```bash
git push origin feature/your-feature-name
```

Then create a Pull Request on GitHub.

## Code Style Guidelines

### Arduino/C++ Code

**Formatting:**
- Use 2 spaces for indentation
- Opening braces on same line
- Clear function and variable names
- Use comments for complex logic

**Example:**
```cpp
void myFunction() {
  // Clear comment explaining purpose
  if (condition) {
    doSomething();
  }
}
```

**Naming Conventions:**
- Functions: `camelCase`
- Variables: `camelCase`
- Constants: `UPPER_CASE`
- Structures: `PascalCase`

### Documentation

**Markdown:**
- Use clear headings
- Include code examples
- Add images where helpful
- Keep line length reasonable

## Testing Requirements

Before submitting a PR, ensure:

- [ ] Code compiles without errors
- [ ] Code compiles without warnings
- [ ] Tested on actual ESP32 hardware
- [ ] GPS functionality verified
- [ ] Buzzer alerts work correctly
- [ ] Serial output is correct
- [ ] No memory leaks or crashes
- [ ] Documentation updated if needed

## Hardware Testing

Test with:
- ESP32 board (specify model)
- NEO-6M GPS module
- Active buzzer
- Multiple camera locations
- Various distances and speeds

## Pull Request Process

### 1. PR Description
Include:
- What changes were made
- Why the changes are needed
- How to test the changes
- Any breaking changes
- Related issues (if any)

### 2. Review Process
- Maintainers will review your PR
- May request changes or clarifications
- Be responsive to feedback
- Update PR as needed

### 3. Approval and Merge
- Once approved, maintainers will merge
- Your contribution will be credited
- Thank you for contributing!

## Regional Database Contributions

### Format for Camera Databases

Create file: `databases/REGION_NAME.txt`

```
# Speed Camera Database - REGION NAME
# Country: COUNTRY
# Last Updated: YYYY-MM-DD
# Contributor: Your Name
# Data Source: Official source or authority

Camera Name, Latitude, Longitude, Speed Limit (km/h)
Main Street Camera, 40.7128, -74.0060, 50
Highway 101 MM45, 40.7589, -73.9851, 80
```

### Database Guidelines
- Use official sources when possible
- Verify accuracy before submitting
- Include data source
- Use decimal degrees format
- Include speed limits in km/h
- Add descriptive names
- Group by area/region

## Feature Implementation Guidelines

### Adding New Features

1. **Discuss First**
   - Open an issue to discuss the feature
   - Get feedback from maintainers
   - Ensure it fits project scope

2. **Plan Implementation**
   - Consider hardware limitations
   - Maintain backward compatibility
   - Keep code clean and maintainable

3. **Update Documentation**
   - Update README if needed
   - Add to HARDWARE_GUIDE if hardware-related
   - Update config.h comments

4. **Example Features Welcome**
   - Additional alert patterns
   - Display integrations (OLED, LED)
   - Data logging to SD card
   - Bluetooth connectivity
   - Web interface
   - Time-based alerts

## Bug Fix Guidelines

### Reporting Bugs

Include:
```
**Description**: Clear description of bug
**Steps to Reproduce**: 
1. Step one
2. Step two
3. Bug occurs

**Expected Behavior**: What should happen
**Actual Behavior**: What actually happens
**Hardware**: ESP32 model, GPS module
**Serial Output**: Paste relevant output
**Photos**: If wiring-related
```

### Fixing Bugs

1. Reproduce the bug
2. Identify root cause
3. Implement fix
4. Test thoroughly
5. Document the fix

## Code Review Checklist

Reviewers will check:

- [ ] Code follows project style
- [ ] Comments are clear and helpful
- [ ] No blocking delays in main loop
- [ ] Memory usage is reasonable
- [ ] No security vulnerabilities
- [ ] Error handling is appropriate
- [ ] Documentation is updated
- [ ] Examples work correctly

## Community Guidelines

### Be Respectful
- Treat everyone with respect
- Be constructive in feedback
- Help others learn
- Welcome newcomers

### Be Patient
- Contributors volunteer their time
- Reviews may take time
- Not all features can be added

### Give Credit
- Acknowledge others' work
- Reference sources
- Thank contributors

## Questions?

- Open an issue for questions
- Tag with [Question]
- Search existing issues first

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Recognition

Contributors will be acknowledged in:
- README.md (major contributions)
- Release notes
- Git commit history

## Thank You!

Your contributions make SpeedZone-Alert better for everyone. Whether you're fixing a typo, adding a feature, or sharing a regional database, your help is appreciated! 🙏

---

**Happy Contributing! 🚀**
