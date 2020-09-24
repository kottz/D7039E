# D7039E
This report-UNSAFE branch is not proof read. 
Write everything here, see workflows below on how to work with the report. 

## Workflow
### Creating a new section
1. Create a new `.tex` file for your section in the `/sections` folder. Use an unique and descriptive name preceded by a chapter where you think it belongs in the format `CHAPTER_descriptive_filename.tex`. For example `APPENDIX_martin_introduction.tex` or `THEORY_robot_arm_kinematics.tex`. If a chapter doesnt exist set it to `UNKNOWN`.
2. Write your section file. Commit and work on it however you want
3. If you have any figures or external files put them in the `/sections/img` folder 
4. This wll be your file to work on and for others to see progress and shouldn't have any merge conflict if you are the only one working on it

### When you feel satisfied with your section
When you're done with your section and feel that it's finished and ready to be reviewed and merged with the report follow this workflow:
1. Push all your changes one last time to `report-UNSAFE`
2. Pull from `report-UNSAFE` just to make sure you have all the changes from others
3. Create a new branch named `REVIEW_chapter`. For example `REVIEW_appendix`.
4. Go to `/chapters` in `report-UNSAFE` branch and paste your section to where you think it belongs, in this case `appendix.tex`. Don't use `\input`. Just paste the raw text
5. Add figures to `/chapters/img`
6. Commit __ONLY ONE__ changed chapter at a time with figures
7. Push your new branch to the repo
8. Go to github and create a new Pull request where
   ```
   Base: report-UNSAFE
   Compare: REVIEW_chapter
   ```
9. Add label `review` and `report` on the right
10. Assign two reviewers. Martin and someone else
11. Publish Pull Request and wait for reviewers to have a look


