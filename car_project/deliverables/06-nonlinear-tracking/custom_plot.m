function custom_plot(x, yCell, plotTitle, xLabel, yLabel, legends)

    % Create the plot
    figure;
    hold on; % Allow multiple plots on the same figure

    % Define colors for each line
    colors = lines(length(yCell)); % Generate a colormap

    % Plot each variable
    for i = 1:length(yCell)
        plot(x, yCell{i}, 'LineWidth', 2, 'Color', colors(i, :));
    end

    % Add grid
    grid on;

    % Add title and labels
    title(plotTitle, 'FontSize', 14, 'FontWeight', 'bold');
    xlabel(xLabel, 'FontSize', 12);
    ylabel(yLabel, 'FontSize', 12);

    % Customize axis appearance
    set(gca, 'FontSize', 12, 'LineWidth', 1);

    % Add legend
    legend(legends, 'Location', 'best');

    % % Adjust axis limits (optional)
    % xlim([min(x) max(x)]);
    % allY = cell2mat(yCell(:));
    % ylim([min(allY) - 0.1 * range(allY), max(allY) + 0.1 * range(allY)]); % Add padding

    hold off; % Release the hold on the plot
end
